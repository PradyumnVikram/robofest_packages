#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>
#include <fstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include "std_msgs/msg/int32_multi_array.hpp"
#include "swarm_control/msg/mother_state.hpp"   // generated from msg/MotherState.msg

constexpr float PI_F = 3.14159265358979323846f;

// Simple quaternion->Euler converter (no Eigen)
void quaternion_to_euler(float x, float y, float z, float w,
                         float &roll, float &pitch, float &yaw)
{
    // Assuming quaternion normalized and in (x,y,z,w)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (w * y - z * x);
    if (std::abs(sinp) >= 1.0f)
        pitch = std::copysign(PI_F / 2.0f, sinp);
    else
        pitch = std::asin(sinp);

    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

class Offboard : public rclcpp::Node
{
public:
    Offboard()
    : Node("offboard"),
      takeoff_mode_(false),
      counter_(0),
      t_(0.1),
      dt_(0.1),
      distance_(0.0f),
      prev_distance_(0.0f),
      vel_(0.0f),
      lock_(0),
      direction_(0),
      vehicle_status_(nullptr),
      vehicle_control_mode_(nullptr),
      vehicle_odometry_(nullptr),
      takeoff_height_(-5.0f),
      rmse_x_(0.0),
      rmse_y_(0.0),
      rmse_z_(0.0),
      rmse_vx_(0.0),
      rmse_vy_(0.0),
      rmse_vz_(0.0),
      integral_error_x_(0.0),
      integral_error_y_(0.0),
      integral_error_z_(0.0),
      acc_mode_(true),
      v_hat_prev_x_(0.0),
      v_hat_prev_y_(0.0),
      v_hat_prev_z_(0.0),
      use_ez_override_(true),
      stable_counter_(0),
      last_pub_x_(NAN), last_pub_y_(NAN), last_pub_z_(NAN),
      last_pub_lock_(-1)
    {
        using namespace std::chrono_literals;

        // QoS profiles
        rclcpp::QoS qos_profile(1);
        qos_profile.best_effort();
        qos_profile.transient_local();

        // Publishers
        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", qos_profile);
        
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", qos_profile);

        trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos_profile);
        // New: publish typed mother state for followers
        mother_state_pub_ = this->create_publisher<swarm_control::msg::MotherState>(
            "/mother_state", rclcpp::QoS(10));    

        // Subscribers
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos_profile,
            std::bind(&Offboard::vehicle_status_callback, this, std::placeholders::_1));

        vehicle_control_mode_sub_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
            "/fmu/out/vehicle_control_mode", qos_profile,
            std::bind(&Offboard::state_callback, this, std::placeholders::_1));

        hand_distance_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/hand_distance", 10,
            std::bind(&Offboard::distance_callback, this, std::placeholders::_1));

        vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos_profile,
            std::bind(&Offboard::vehicle_odometry_callback, this, std::placeholders::_1));

        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(0.1),
            std::bind(&Offboard::command_loop, this));

        // CSV header
        std::ofstream file("state_data_task_4.csv", std::ios::out | std::ios::trunc);
        if (file.is_open()) {
            file << "time,pos_x,pos_y,pos_z,ideal_x,ideal_y,ideal_z,"
                 << "error_x,error_y,error_z,"
                 << "vel_x,vel_y,vel_z,"
                 << "ideal_vx,ideal_vy,ideal_vz,"
                 << "error_vx,error_vy,error_vz,"
                 << "roll,pitch,yaw,roll_rate,pitch_rate,yaw_rate\n";
            file.close();
        }
    }

    double get_t() const { return t_; }
    double get_rmse_x() const { return rmse_x_; }
    double get_rmse_y() const { return rmse_y_; }
    double get_rmse_z() const { return rmse_z_; }
    double get_rmse_vx() const { return rmse_vx_; }
    double get_rmse_vy() const { return rmse_vy_; }
    double get_rmse_vz() const { return rmse_vz_; }

private:
    // Callbacks
    void state_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
    {
        vehicle_control_mode_ = msg;
    }

    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        vehicle_status_ = msg;
    }

    void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        vehicle_odometry_ = msg;
    }

    void distance_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        prev_distance_ = distance_;
        if (!msg->data.empty()) {
            // msg->data[0] = distance, [1] = lock, [2] = direction
            distance_ = static_cast<float>(msg->data[0]) / 100.0f;
            vel_ = (distance_ - prev_distance_) * 30.0f;
            if (msg->data.size() > 1) {
                lock_ = msg->data[1];
            }
            if (msg->data.size() > 2) {
                direction_ = msg->data[2];
            }
        }
    }
     // Publish mother state for followers using the typed message
    void publish_mother_state(bool force_publish = false)
    {
        if (!mother_state_pub_) return;
        if (!vehicle_odometry_) return;

        // Convert quaternion to yaw (note: px4_msgs VehicleOdometry q layout is [w,x,y,z] usually)
        float qw = vehicle_odometry_->q[0];
        float qx = vehicle_odometry_->q[1];
        float qy = vehicle_odometry_->q[2];
        float qz = vehicle_odometry_->q[3];
        float roll, pitch, yaw;
        quaternion_to_euler(qx, qy, qz, qw, roll, pitch, yaw);

        swarm_control::msg::MotherState msg;
        msg.x = static_cast<float>(vehicle_odometry_->position[0]);
        msg.y = static_cast<float>(vehicle_odometry_->position[1]);
        msg.z = static_cast<float>(vehicle_odometry_->position[2]);
        msg.yaw = yaw;
        msg.distance = distance_;
        msg.lock = lock_;
        msg.direction = direction_;
        msg.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);

        // Determine at_goal boolean using simple stability test:
        // if lock_ != 0 -> mother is (likely) moving, mark at_goal = false
        bool at_goal = false;
        double vx = vehicle_odometry_->velocity[0];
        double vy = vehicle_odometry_->velocity[1];
        double vz = vehicle_odometry_->velocity[2];
        double vel_norm = std::sqrt(vx*vx + vy*vy + vz*vz);

        double px = vehicle_odometry_->position[0];
        double py = vehicle_odometry_->position[1];
        double pz = vehicle_odometry_->position[2];

        // If lock_ != 0, mother is executing a gesture movement -> not at goal
        if (lock_ != 0) {
            stable_counter_ = 0;
            at_goal = false;
        } else {
            // If velocity small and position does not change much since last publishes, increment stable_counter_
            double pos_delta = NAN;
            if (!std::isnan(last_pub_x_)) {
                double dx = px - last_pub_x_;
                double dy = py - last_pub_y_;
                double dz = pz - last_pub_z_;
                pos_delta = std::sqrt(dx*dx + dy*dy + dz*dz);
            } else {
                pos_delta = 0.0;
            }

            // thresholds - tune as needed
            const double VEL_THRESH = 0.08;   // m/s
            const double POS_DELTA_THRESH = 0.05; // m
            if (vel_norm < VEL_THRESH && pos_delta < POS_DELTA_THRESH) {
                stable_counter_++;
            } else {
                stable_counter_ = 0;
            }

            // require a few stable cycles to declare at_goal true
            if (stable_counter_ >= 3) {
                at_goal = true;
            } else {
                at_goal = false;
            }
        }

        msg.at_goal = at_goal;
         // Optionally avoid spamming if pose hasn't changed much
        if (!force_publish) {
            if (!std::isnan(last_pub_x_) &&
                std::fabs(msg.x - last_pub_x_) < 0.01 &&
                std::fabs(msg.y - last_pub_y_) < 0.01 &&
                std::fabs(msg.z - last_pub_z_) < 0.01 &&
                msg.lock == last_pub_lock_) {
                // No significant change, skip publish
                return;
            }
        }
        mother_state_pub_->publish(msg);
        last_pub_x_ = msg.x;
        last_pub_y_ = msg.y;
        last_pub_z_ = msg.z;
        last_pub_lock_ = msg.lock;
    }

    // Offboard heartbeat
    void offboard_control_heartbeat_signal_publisher()
    {
        auto msg = px4_msgs::msg::OffboardControlMode();
        msg.position = takeoff_mode_;
        msg.velocity = false;
        msg.acceleration = acc_mode_;
        msg.attitude = false;
        msg.body_rate = false;
        msg.thrust_and_torque = false;
        msg.direct_actuator = false;

        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_mode_pub_->publish(msg);
    }

    // Vehicle commands
    void engage_offboard_mode()
    {
        uint8_t instance_num = 1;
        auto msg = px4_msgs::msg::VehicleCommand();
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        msg.param1 = 1.0f;
        msg.param2 = 6.0f;
        msg.target_system = instance_num;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);
    }

    void arm()
    {
        uint8_t instance_num = 1;
        auto msg = px4_msgs::msg::VehicleCommand();
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.param1 = 1.0f;
        msg.param2 = 0.0f;
        msg.param3 = 0.0f;
        msg.param4 = 0.0f;
        msg.param5 = 0.0f;
        msg.param6 = 0.0f;
        msg.param7 = 0.0f;
        msg.target_system = instance_num;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);
    }

    void disarm()
    {
        uint8_t instance_num = 1;
        auto msg = px4_msgs::msg::VehicleCommand();
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.param1 = 0.0f;
        msg.param2 = 0.0f;
        msg.param3 = 0.0f;
        msg.param4 = 0.0f;
        msg.param5 = 0.0f;
        msg.param6 = 0.0f;
        msg.param7 = 0.0f;
        msg.target_system = instance_num;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);
    }

    void takeoff()
    {
        uint8_t instance_num = 1;
        auto msg = px4_msgs::msg::VehicleCommand();
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
        msg.param1 = 0.0f;
        msg.param2 = 0.0f;
        msg.param3 = 0.0f;
        msg.param4 = 0.0f;
        msg.param5 = 0.0f;
        msg.param6 = 0.0f;
        msg.param7 = 0.0f;
        msg.target_system = instance_num;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);
    }

    // Trajectory helpers
    void publish_trajectory_wpt(float x, float y, float z)
    {
        auto msg = px4_msgs::msg::TrajectorySetpoint();
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
        msg.yaw = 1.57079f;
        msg.yawspeed = NAN;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_pub_->publish(msg);
    }

    void publish_trajectory_v(float vx, float vy, float vz)
    {
        auto msg = px4_msgs::msg::TrajectorySetpoint();
        msg.position[0] = NAN;
        msg.position[1] = NAN;
        msg.position[2] = NAN;
        msg.velocity[0] = vx;
        msg.velocity[1] = vy;
        msg.velocity[2] = vz;
        msg.acceleration[0] = NAN;
        msg.acceleration[1] = NAN;
        msg.acceleration[2] = NAN;
        msg.jerk[0] = NAN;
        msg.jerk[1] = NAN;
        msg.jerk[2] = NAN;
        msg.yaw = 1.57079f;
        msg.yawspeed = NAN;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_pub_->publish(msg);
    }

    void publish_trajectory_a(float ax, float ay, float az)
    {
        auto msg = px4_msgs::msg::TrajectorySetpoint();
        msg.position[0] = NAN;
        msg.position[1] = NAN;
        msg.position[2] = NAN;
        msg.velocity[0] = NAN;
        msg.velocity[1] = NAN;
        msg.velocity[2] = NAN;
        msg.acceleration[0] = ax;
        msg.acceleration[1] = ay;
        msg.acceleration[2] = az;
        msg.jerk[0] = NAN;
        msg.jerk[1] = NAN;
        msg.jerk[2] = NAN;
        msg.yaw = 1.57079f;
        msg.yawspeed = NAN;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_pub_->publish(msg);
    }

    // Error and controllers
    void compute_error(double &error_x, double &error_y, double &error_z,
                       double &error_vx, double &error_vy, double &error_vz)
    {
        if (!vehicle_odometry_) {
            error_x = error_y = error_z = 0.0;
            error_vx = error_vy = error_vz = 0.0;
            return;
        }

        if (direction_ != 0) {
            // direction true branch
            error_x = x_ref_ - vehicle_odometry_->position[0];
            error_z = z_ref_ - vehicle_odometry_->position[2];
            error_y = static_cast<double>(distance_) - vehicle_odometry_->position[1];

            error_vx = 0.0 - vehicle_odometry_->velocity[0];
            error_vy = static_cast<double>(vel_) - vehicle_odometry_->velocity[1];
            error_vz = 0.0 - vehicle_odometry_->velocity[2];
        } else {
            // direction false branch
            error_x = static_cast<double>(distance_) - vehicle_odometry_->position[0];
            error_z = z_ref_ - vehicle_odometry_->position[2];
            error_y = y_ref_ - vehicle_odometry_->position[1];

            error_vx = static_cast<double>(vel_) - vehicle_odometry_->velocity[0];
            error_vy = 0.0 - vehicle_odometry_->velocity[1];
            error_vz = 0.0 - vehicle_odometry_->velocity[2];
        }

        rmse_x_ += error_x * error_x;
        rmse_y_ += error_y * error_y;
        rmse_z_ += error_z * error_z;

        rmse_vx_ += error_vx * error_vx;
        rmse_vy_ += error_vy * error_vy;
        rmse_vz_ += error_vz * error_vz;
    }

    void tracking_pp_controller(double &ax, double &ay, double &az)
    {
        double ex, ey, ez, evx, evy, evz;
        compute_error(ex, ey, ez, evx, evy, evz);
         // Apply override to z-error if enabled and odometry is available.
        // This makes ez equal to -z (as in the Python parity comment).
        if (use_ez_override_ && vehicle_odometry_) {
            ez = 0.0 - vehicle_odometry_->position[2];
        }

        ax = 3.0 * ex + 5.0 * evx;
        ay = 3.0 * ey + 5.0 * evy;
        az = 3.0 * ez + 5.0 * evz;
    }

    // Main loop
    void command_loop()
    {
        offboard_control_heartbeat_signal_publisher();

        RCLCPP_INFO(this->get_logger(), "takeoff_mode: %s, counter: %d",
                    takeoff_mode_ ? "true" : "false", counter_);
        counter_++;

        if (counter_ < 100) {
            // keep sending heartbeat before enabling offboard
            publish_mother_state();
            return;
        } else if (counter_ == 100) {
            engage_offboard_mode();
            arm();
            takeoff_mode_ = true;
        }

        if (takeoff_mode_) {
            RCLCPP_INFO(this->get_logger(), "Takeoff phase...");
            publish_trajectory_wpt(0.0f, 0.0f, -5.0f);

            if (vehicle_odometry_) {
                double z = vehicle_odometry_->position[2];
                if (z >= -5.0 - 0.1 && z <= -5.0 + 0.1) {
                    z_ref_ = vehicle_odometry_->position[2];
                    y_ref_ = vehicle_odometry_->position[1];
                    x_ref_ = vehicle_odometry_->position[0];
                    takeoff_mode_ = false;
                     // Publish mother state at the moment we finish takeoff and set refs
                    publish_mother_state(true);
                }
            }
        }

        if (!takeoff_mode_ && counter_ > 100) {
            if (lock_ != 0) {
                double ax, ay, az;
                tracking_pp_controller(ax, ay, az);
                publish_trajectory_a(static_cast<float>(ax),
                                     static_cast<float>(ay),
                                     static_cast<float>(az));
            } else {
                publish_trajectory_v(0.0f, 0.0f, 0.0f);
            }
        }

        if (vehicle_odometry_ && !takeoff_mode_ && counter_ > 100) {
            RCLCPP_INFO(this->get_logger(), "t: %.3f", t_);

            double ex, ey, ez, evx, evy, evz;
            compute_error(ex, ey, ez, evx, evy, evz);

            
            t_ += 0.1;
            publish_mother_state();
        }

        RCLCPP_INFO(this->get_logger(), " ");
    }

    // Members
    bool takeoff_mode_;
    int counter_;
    double t_;
    double dt_;

    float distance_;
    float prev_distance_;
    float vel_;
    int32_t lock_;
    int32_t direction_;

    double x_ref_{0.0}, y_ref_{0.0}, z_ref_{0.0};
    float takeoff_height_;

    double rmse_x_, rmse_y_, rmse_z_;
    double rmse_vx_, rmse_vy_, rmse_vz_;

    double integral_error_x_, integral_error_y_, integral_error_z_;

    bool acc_mode_;

    double v_hat_prev_x_, v_hat_prev_y_, v_hat_prev_z_;

    bool use_ez_override_;

    px4_msgs::msg::VehicleStatus::SharedPtr vehicle_status_;
    px4_msgs::msg::VehicleControlMode::SharedPtr vehicle_control_mode_;
    px4_msgs::msg::VehicleOdometry::SharedPtr vehicle_odometry_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<swarm_control::msg::MotherState>::SharedPtr mother_state_pub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr hand_distance_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;

    rclcpp::TimerBase::SharedPtr timer_; 
    // for at_goal detection
    int stable_counter_;
    double last_pub_x_, last_pub_y_, last_pub_z_;
    int last_pub_lock_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Offboard>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        std::printf("Exception: %s\n", e.what());
    }

    double t = node->get_t();
    if (t > 0.0) {
        std::printf("rmse_x = %f\n", std::sqrt(node->get_rmse_x() / t));
        std::printf("rmse_y = %f\n", std::sqrt(node->get_rmse_y() / t));
        std::printf("rmse_z = %f\n", std::sqrt(node->get_rmse_z() / t));

        std::printf("rmse_vx = %f\n", std::sqrt(node->get_rmse_vx() / t));
        std::printf("rmse_vy = %f\n", std::sqrt(node->get_rmse_vy() / t));
        std::printf("rmse_vz = %f\n", std::sqrt(node->get_rmse_vz() / t));
    }
    rclcpp::shutdown();
    return 0;
}
