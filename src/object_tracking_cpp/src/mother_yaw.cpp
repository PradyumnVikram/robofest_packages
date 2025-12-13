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

#include "std_msgs/msg/float32.hpp"

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
      ready_to_yaw_(false),
      counter_(0),
      t_(0.1),
      dt_(0.1),
      distance_(0.0f),
      prev_distance_(0.0f),
      vel_(0.0f),
      lock_(0),
      direction_(0),
      swarm_call_(0),
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
      acc_mode_(false),   // not using accel setpoints for now
      v_hat_prev_x_(0.0),
      v_hat_prev_y_(0.0),
      v_hat_prev_z_(0.0),
      use_ez_override_(true),
      stable_counter_(0),
      last_pub_x_(NAN), last_pub_y_(NAN), last_pub_z_(NAN),
      last_pub_lock_(-1),
      msg_yaw_(0.0f),
      yaw_rate_(0.0f),
      x_hover_(0.0),
      y_hover_(0.0),
      z_hover_(0.0)
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

        // Subscribers
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos_profile,
            std::bind(&Offboard::vehicle_status_callback, this, std::placeholders::_1));

        vehicle_control_mode_sub_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
            "/fmu/out/vehicle_control_mode", qos_profile,
            std::bind(&Offboard::state_callback, this, std::placeholders::_1));

        vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos_profile,
            std::bind(&Offboard::vehicle_odometry_callback, this, std::placeholders::_1));

        subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/waist_angle", 10, std::bind(&Offboard::yaw_callback, this, std::placeholders::_1));

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
    void yaw_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        msg_yaw_ = msg->data;
    }

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

    // Offboard heartbeat: always position mode
    void offboard_control_heartbeat_signal_publisher()
    {
        auto msg = px4_msgs::msg::OffboardControlMode();
        msg.position = true;       // always control position (hover + yaw)
        msg.velocity = false;
        msg.acceleration = false;
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
        msg.param2 = 6.0f;  // PX4: custom mode 6 = Offboard [web:11]
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
        px4_msgs::msg::TrajectorySetpoint msg{};
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

    float abs_val(float x){
      return x < 0 ? -x : x;
    }

    // Main loop
    void command_loop()
    {
        offboard_control_heartbeat_signal_publisher();

        RCLCPP_INFO(this->get_logger(), "takeoff_mode: %s, ready_to_yaw: %s, counter: %d",
                    takeoff_mode_ ? "true" : "false",
                    ready_to_yaw_ ? "true" : "false",
                    counter_);

        counter_++;

        // 1) Pre-offboard: stream setpoints for PX4 to trust offboard
        if (counter_ < 100) {
            // Just stream hover at (0,0,-5) (ignored until in offboard) [web:11]
            publish_trajectory_wpt(0.0f, 0.0f, takeoff_height_);
            return;
        } else if (counter_ == 100) {
            engage_offboard_mode();
            arm();
            takeoff_mode_ = true;
        }

        // 2) Takeoff phase: go to (0,0,-5)
        if (takeoff_mode_) {
            RCLCPP_INFO(this->get_logger(), "Takeoff phase...");
            publish_trajectory_wpt(0.0f, 0.0f, takeoff_height_);

            if (vehicle_odometry_) {
                double z = vehicle_odometry_->position[2];
                if (z >= takeoff_height_ - 0.1 && z <= takeoff_height_ + 0.1) {
                    z_ref_ = vehicle_odometry_->position[2];
                    y_ref_ = vehicle_odometry_->position[1];
                    x_ref_ = vehicle_odometry_->position[0];

                    // Set hover point
                    x_hover_ = x_ref_;
                    y_hover_ = y_ref_;
                    z_hover_ = z_ref_;

                    takeoff_mode_ = false;
                    ready_to_yaw_ = true;
                    RCLCPP_INFO(this->get_logger(), "Reached takeoff height, switching to yaw control");
                }
            }
            return;
        }

        // 3) Hover + yaw control phase
        if (!takeoff_mode_ && counter_ > 100 && ready_to_yaw_) {
            // Maintain position, apply yaw rate from waist_angle [web:51][web:65]
            float yawspeed_sp = 0.0f;

            if (abs_val(msg_yaw_) > 10.0f) {
                yawspeed_sp = (msg_yaw_ < 0) ? -0.1f : 0.1f;
            }

            px4_msgs::msg::TrajectorySetpoint msg{};
            msg.position[0] = x_hover_;
            msg.position[1] = y_hover_;
            msg.position[2] = z_hover_;
            msg.velocity[0] = NAN;
            msg.velocity[1] = NAN;
            msg.velocity[2] = NAN;
            msg.acceleration[0] = NAN;
            msg.acceleration[1] = NAN;
            msg.acceleration[2] = NAN;
            msg.jerk[0] = NAN;
            msg.jerk[1] = NAN;
            msg.jerk[2] = NAN;

            // Open-loop yaw-rate control: yaw = NAN, yawspeed set [web:51]
            msg.yaw = NAN;
            msg.yawspeed = yawspeed_sp;

            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            trajectory_pub_->publish(msg);
        }
    }

    // Members
    bool takeoff_mode_;
    bool ready_to_yaw_;
    int counter_;
    double t_;
    double dt_;
    float msg_yaw_;
    float yaw_rate_;

    float distance_;
    float prev_distance_;
    float vel_;
    int32_t lock_;
    int32_t direction_;
    int32_t swarm_call_;

    double x_ref_{0.0}, y_ref_{0.0}, z_ref_{0.0};
    float takeoff_height_;
    double x_hover_, y_hover_, z_hover_;

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

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;

    rclcpp::TimerBase::SharedPtr timer_;

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
