#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include "swarm_control/msg/mother_state.hpp"

using namespace std::chrono_literals;

class SwarmFollower : public rclcpp::Node
{
public:
    SwarmFollower()
    : Node("swarm_follower"),
      state_(State::PRE),
      have_offset_(false),
      last_mother_lock_(0),
      publish_hz_(10.0),
      arrival_tolerance_m_(0.30)
    {
        // parameters (can be overridden at runtime / launch)
        this->declare_parameter<std::string>("mother_state_topic", "/mother_state");
        this->declare_parameter<std::string>("odom_topic", "fmu/out/vehicle_odometry");
        this->declare_parameter<double>("publish_rate", publish_hz_);
        this->declare_parameter<double>("arrival_tolerance_m", arrival_tolerance_m_);

        this->get_parameter("mother_state_topic", mother_state_topic_);
        this->get_parameter("odom_topic", odom_topic_);
        this->get_parameter("publish_rate", publish_hz_);
        this->get_parameter("arrival_tolerance_m", arrival_tolerance_m_);

        publish_period_ = std::chrono::duration<double>(1.0 / publish_hz_);

        // QoS
        rclcpp::QoS qos(10);
        qos.best_effort();

        // Publishers — relative topics so namespace remapping works per-drone
        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", qos);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/px4_1/fmu/in/vehicle_command", qos);
        trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", qos);

        // Subscribers
        mother_state_sub_ = this->create_subscription<swarm_control::msg::MotherState>(
            mother_state_topic_, 10,
            std::bind(&SwarmFollower::mother_state_cb, this, std::placeholders::_1));

        vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            odom_topic_, qos,
            std::bind(&SwarmFollower::odom_cb, this, std::placeholders::_1));

        // Timer
        timer_ = this->create_wall_timer(publish_period_, std::bind(&SwarmFollower::timer_cb, this));

        RCLCPP_INFO(this->get_logger(),
                    "SwarmFollower started. mother_state_topic=%s, odom_topic=%s, rate=%.2fHz, tol=%.2fm",
                    mother_state_topic_.c_str(), odom_topic_.c_str(), publish_hz_, arrival_tolerance_m_);
    }

private:
    enum class State { PRE, WAIT_LEADER, MOVE_TO_REL, HOLD };

    // Callbacks
    void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        vehicle_odometry_ = msg;
    }

    void mother_state_cb(const swarm_control::msg::MotherState::SharedPtr msg)
    {
        // copy leader state
        mother_x_ = msg->x;
        mother_y_ = msg->y;
        mother_z_ = msg->z;
        mother_yaw_ = msg->yaw;
        mother_lock_ = msg->lock;
        mother_at_goal_ = msg->at_goal;
        mother_direction_ = msg->direction;
        mother_distance_ = msg->distance;

        // state transitions driven by mother lock/at_goal
        if (state_ == State::PRE) {
            if (mother_lock_ != 0) {
                state_ = State::WAIT_LEADER;
                RCLCPP_INFO(this->get_logger(), "Mother gesture detected -> WAIT_LEADER");
            }
        } else if (state_ == State::WAIT_LEADER) {
            if (mother_at_goal_ && mother_lock_ == 0) {
                if (have_offset_) {
                    state_ = State::MOVE_TO_REL;
                    RCLCPP_INFO(this->get_logger(), "Mother reached goal -> MOVE_TO_REL");
                } else {
                    RCLCPP_WARN(this->get_logger(), "No stored offset when mother reached goal; staying PRE");
                    state_ = State::PRE;
                }
            }
        } else if (state_ == State::MOVE_TO_REL || state_ == State::HOLD) {
            // if mother starts moving again, go back to WAIT_LEADER
            if (mother_lock_ != 0 && !mother_at_goal_) {
                state_ = State::WAIT_LEADER;
                RCLCPP_INFO(this->get_logger(), "Mother moved again -> WAIT_LEADER");
            }
        }
    }

    // Helpers
    void publish_offboard_heartbeat()
    {
        px4_msgs::msg::OffboardControlMode msg;
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.direct_actuator = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_mode_pub_->publish(msg);
    }

    void publish_trajectory_wpt(float x, float y, float z)
    {
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.position[0] = x;
        msg.position[1] = y;
        msg.position[2] = z;
        msg.velocity[0] = NAN;
        msg.velocity[1] = NAN;
        msg.velocity[2] = NAN;
        msg.acceleration[0] = NAN;
        msg.acceleration[1] = NAN;
        msg.acceleration[2] = NAN;
        msg.jerk[0] = NAN;
        msg.jerk[1] = NAN;
        msg.jerk[2] = NAN;
        msg.yaw = NAN;
        msg.yawspeed = NAN;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_pub_->publish(msg);
    }

    double dist3d(double ax, double ay, double az, double bx, double by, double bz) const
    {
        double dx = ax - bx;
        double dy = ay - by;
        double dz = az - bz;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    // Main periodic logic
    void timer_cb()
    {
        publish_offboard_heartbeat();

        // require both local odom and mother state
        if (!vehicle_odometry_) {
            RCLCPP_DEBUG(this->get_logger(), "waiting for local odom...");
            return;
        }
        if (std::isnan(mother_x_) && std::isnan(mother_y_) && std::isnan(mother_z_)) {
            RCLCPP_DEBUG(this->get_logger(), "waiting for mother state...");
            return;
        }

        // PRE: while mother is steady (at_goal && lock==0), continuously store offset
        if (state_ == State::PRE) {
            if (mother_at_goal_ && mother_lock_ == 0) {
                stored_offset_x_ = static_cast<double>(vehicle_odometry_->position[0]) - static_cast<double>(mother_x_);
                stored_offset_y_ = static_cast<double>(vehicle_odometry_->position[1]) - static_cast<double>(mother_y_);
                stored_offset_z_ = static_cast<double>(vehicle_odometry_->position[2]) - static_cast<double>(mother_z_);
                have_offset_ = true;
                RCLCPP_DEBUG(this->get_logger(), "stored offset updated (pre-move): (%.2f, %.2f, %.2f)",
                             stored_offset_x_, stored_offset_y_, stored_offset_z_);
            }
            return; // do not command motion in PRE
        }

        // WAIT_LEADER: leader is moving -> do nothing, wait for mother_at_goal (transition handled in callback)
        if (state_ == State::WAIT_LEADER) {
            return;
        }

        // MOVE_TO_REL: mother has finished move, command follower to mother + offset until arrival
        if (state_ == State::MOVE_TO_REL && have_offset_) {
            double tx = static_cast<double>(mother_x_) + stored_offset_x_;
            double ty = static_cast<double>(mother_y_) + stored_offset_y_;
            double tz = static_cast<double>(mother_z_) + stored_offset_z_;

            publish_trajectory_wpt(static_cast<float>(tx), static_cast<float>(ty), static_cast<float>(tz));
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Moving to formation target (%.2f, %.2f, %.2f)", tx, ty, tz);

            // check arrival
            double curx = vehicle_odometry_->position[0];
            double cury = vehicle_odometry_->position[1];
            double curz = vehicle_odometry_->position[2];

            double d = dist3d(curx, cury, curz, tx, ty, tz);
            if (d <= arrival_tolerance_m_) {
                state_ = State::HOLD;
                RCLCPP_INFO(this->get_logger(), "Arrived at relative target -> HOLD");
            }
            return;
        }

        // HOLD: keep publishing the relative waypoint (mother + offset) to maintain formation
        if (state_ == State::HOLD && have_offset_) {
            double hold_x = static_cast<double>(mother_x_) + stored_offset_x_;
            double hold_y = static_cast<double>(mother_y_) + stored_offset_y_;
            double hold_z = static_cast<double>(mother_z_) + stored_offset_z_;
            publish_trajectory_wpt(static_cast<float>(hold_x), static_cast<float>(hold_y), static_cast<float>(hold_z));
            return;
        }
    }

    // Members / params
    std::string mother_state_topic_;
    std::string odom_topic_;
    double publish_hz_;
    double arrival_tolerance_m_;
    std::chrono::duration<double> publish_period_;

    // publishers & subs
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;

    rclcpp::Subscription<swarm_control::msg::MotherState>::SharedPtr mother_state_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // leader state
    double mother_x_ = NAN;
    double mother_y_ = NAN;
    double mother_z_ = NAN;
    double mother_yaw_ = NAN;
    int mother_lock_ = 0;
    bool mother_at_goal_ = false;
    int mother_direction_ = 0;
    float mother_distance_ = NAN;

    // local odom
    px4_msgs::msg::VehicleOdometry::SharedPtr vehicle_odometry_;

    // offsets & state
    double stored_offset_x_ = 0.0;
    double stored_offset_y_ = 0.0;
    double stored_offset_z_ = 0.0;
    bool have_offset_;
    int last_mother_lock_;
    State state_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SwarmFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}