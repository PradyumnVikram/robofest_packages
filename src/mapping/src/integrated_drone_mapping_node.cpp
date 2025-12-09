/*
Integrated PX4 Drone Flight and Occupancy Grid Mapping Node (C++)
Combines: Flight Control + LiDAR Subscription + Real-time Occupancy Grid Mapping
*/

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <array>
#include <vector>
#include <cmath>
#include <chrono>
#include <memory>
#include <filesystem>
#include <fstream>

class IntegratedDroneMappingNode : public rclcpp::Node {
public:
    IntegratedDroneMappingNode() : Node("integrated_drone_mapping_node") {
        init_parameters();
        init_publishers();
        init_subscribers();
        init_timers();
        
        RCLCPP_INFO(get_logger(), "=== Integrated Drone Mapping Node Initialized ===");
        RCLCPP_INFO(get_logger(), "Grid: %dx%d cells | Mapping update rate: 3 Hz", 
                   GRID_WIDTH_CELLS, GRID_HEIGHT_CELLS);
    }

private:
    // ========== FLIGHT CONTROL PARAMETERS ==========
    std::string flight_state_ = "INIT";
    int counter_ = 0;
    bool mapping_begin_ = false;
    
    const float TAKEOFF_HEIGHT = -1.0f;
    const float FORWARD_DISTANCE = -5.0f;
    const float POSITION_THRESHOLD = 0.15f;
    const float HOVER_TIME = 3.0f;
    
    struct Waypoint {
        float x, y, z;
    };
    Waypoint current_waypoint_{0,0,0};
    Waypoint takeoff_position_{0,0,0};
    
    px4_msgs::msg::VehicleOdometry vehicle_odometry_{};
    rclcpp::Time hover_start_time_;

    // ========== OCCUPANCY GRID PARAMETERS ==========
    static constexpr float GRID_WIDTH_M = 10.0f;
    static constexpr float GRID_HEIGHT_M = 10.0f;
    static constexpr float RESOLUTION = 0.1f;
    static constexpr int GRID_WIDTH_CELLS = static_cast<int>(GRID_WIDTH_M / RESOLUTION);
    static constexpr int GRID_HEIGHT_CELLS = static_cast<int>(GRID_HEIGHT_M / RESOLUTION);
    static constexpr int GRID_ORIGIN_X_CELL = GRID_WIDTH_CELLS / 2;
    static constexpr int GRID_ORIGIN_Y_CELL = GRID_HEIGHT_CELLS / 2;
    
    static constexpr float LOG_ODDS_OCC = 2.2f;
    static constexpr float LOG_ODDS_FREE = -0.7f;
    static constexpr float LOG_ODDS_MAX = 5.0f;
    static constexpr float LOG_ODDS_MIN = -5.0f;
    static constexpr float MAX_RANGE = 20.0f;
    
    std::vector<float> occupancy_grid_;
    sensor_msgs::msg::LaserScan latest_scan_data_;
    bool has_scan_data_ = false;
    bool has_odometry_ = false;
    int scan_count_ = 0;
    int mapping_iterations_ = 0;
    
    std::filesystem::path output_dir_;

    // ========== ROS2 INTERFACES ==========
    rclcpp::QoS qos_profile_{rclcpp::KeepLast(1)};
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr mapping_timer_;

    void init_parameters() {
        qos_profile_ = rclcpp::QoS(rclcpp::KeepLast(1))
            .best_effort()
            .transient_local();
        
        occupancy_grid_.resize(GRID_HEIGHT_CELLS * GRID_WIDTH_CELLS, 0.0f);
        output_dir_ = std::filesystem::path(std::getenv("HOME")) / "occupancy_maps";
        std::filesystem::create_directories(output_dir_);
    }

    void init_publishers() {
        offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", qos_profile_);
        vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", qos_profile_);
        trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos_profile_);
    }

    void init_subscribers() {
        odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos_profile_,
            std::bind(&IntegratedDroneMappingNode::vehicle_odometry_callback, this, std::placeholders::_1));
        
        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::QoS(10).best_effort(),
            std::bind(&IntegratedDroneMappingNode::laser_callback, this, std::placeholders::_1));
    }

    void init_timers() {
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&IntegratedDroneMappingNode::control_loop, this));
        mapping_timer_ = create_wall_timer(
            std::chrono::milliseconds(333), 
            std::bind(&IntegratedDroneMappingNode::mapping_loop, this));
    }

    // ============= CALLBACKS =============
    void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        vehicle_odometry_ = *msg;
        has_odometry_ = true;
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_data_ = *msg;
        has_scan_data_ = true;
        
        if (scan_count_ % 10 == 0) {
            RCLCPP_INFO(get_logger(), "LiDAR: %zu beams, Range: [%.2f, %.2f]m",
                       msg->ranges.size(), msg->range_min, msg->range_max);
        }
        scan_count_++;
    }

    // ============= FLIGHT CONTROL =============
    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg;
        msg.position = true;
        msg.timestamp = now().nanoseconds() / 1000;
        offboard_mode_pub_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f) {
        px4_msgs::msg::VehicleCommand msg;
        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);
    }

    void engage_offboard_mode() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
        RCLCPP_INFO(get_logger(), "Engaging Offboard Mode");
    }

    void arm() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        RCLCPP_INFO(get_logger(), "Arming Vehicle");
    }

    void disarm() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
        RCLCPP_INFO(get_logger(), "Disarming Vehicle");
    }

    void publish_position_setpoint(float x, float y, float z, float yaw = 0.0f) {
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.position[0] = x;
        msg.position[1] = y;
        msg.position[2] = z;
        msg.velocity[0] = NAN;
        msg.velocity[1] = NAN;
        msg.velocity[2] = NAN;
        msg.yaw = yaw;
        msg.timestamp = now().nanoseconds() / 1000;
        trajectory_pub_->publish(msg);
    }

    float get_position_error(float target_x, float target_y, float target_z) {
        if (!has_odometry_) return INFINITY;
        float dx = target_x - vehicle_odometry_.position[0];
        float dy = target_y - vehicle_odometry_.position[1];
        float dz = target_z - vehicle_odometry_.position[2];
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    bool at_position(float target_x, float target_y, float target_z) {
        return get_position_error(target_x, target_y, target_z) < POSITION_THRESHOLD;
    }

    // ============= OCCUPANCY GRID MAPPING =============
    void world_to_grid(float wx, float wy, int& gx, int& gy) {
        gx = static_cast<int>((wx / RESOLUTION) + GRID_ORIGIN_X_CELL);
        gy = static_cast<int>((wy / RESOLUTION) + GRID_ORIGIN_Y_CELL);
    }

    bool is_in_grid_bounds(int gx, int gy) {
        return gx >= 0 && gx < GRID_WIDTH_CELLS && gy >= 0 && gy < GRID_HEIGHT_CELLS;
    }

    std::vector<std::array<int, 2>> bresenham_line(int x0, int y0, int x1, int y1) {
        std::vector<std::array<int, 2>> cells;
        int dx = std::abs(x1 - x0);
        int dy = -std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx + dy;
        
        int x = x0, y = y0;
        while (true) {
            cells.push_back({x, y});
            if (x == x1 && y == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x += sx; }
            if (e2 <= dx) { err += dx; y += sy; }
        }
        return cells;
    }

    int update_occupancy_grid(float robot_x, float robot_y, float robot_theta) {
        int valid_updates = 0;
        
        int robot_gx, robot_gy;
        world_to_grid(robot_x, robot_y, robot_gx, robot_gy);
        
        float angle = latest_scan_data_.angle_min;
        for (float range_m : latest_scan_data_.ranges) {
            if (std::isinf(range_m) || std::isnan(range_m)) {
                range_m = MAX_RANGE;
            }
            
            float global_angle = robot_theta + angle;
            float hit_x = robot_x - (range_m * std::sin(global_angle));
            float hit_y = robot_y - (range_m * std::cos(global_angle));
            
            int hit_gx, hit_gy;
            world_to_grid(hit_x, hit_y, hit_gx, hit_gy);
            
            auto line_cells = bresenham_line(robot_gx, robot_gy, hit_gx, hit_gy);
            
            // Free cells
            for (size_t i = 0; i < line_cells.size() - 1; ++i) {
                auto [gx, gy] = line_cells[i];
                if (is_in_grid_bounds(gx, gy)) {
                    int idx = gy * GRID_WIDTH_CELLS + gx;
                    occupancy_grid_[idx] += LOG_ODDS_FREE;
                    occupancy_grid_[idx] = std::max(LOG_ODDS_MIN, occupancy_grid_[idx]);
                }
            }
            
            // Occupied cell
            if (is_in_grid_bounds(hit_gx, hit_gy)) {
                int idx = hit_gy * GRID_WIDTH_CELLS + hit_gx;
                occupancy_grid_[idx] += LOG_ODDS_OCC;
                occupancy_grid_[idx] = std::min(LOG_ODDS_MAX, occupancy_grid_[idx]);
                valid_updates++;
            }
            
            angle += latest_scan_data_.angle_increment;
        }
        return valid_updates;
    }

    // ============= MAIN LOOPS =============
    void control_loop() {
        publish_offboard_control_mode();
        counter_++;
        
        if (flight_state_ == "INIT") {
            if (counter_ < 100) return;
            engage_offboard_mode();
            arm();
            flight_state_ = "ARMING";
            RCLCPP_INFO(get_logger(), "State: ARMING");
            
        } else if (flight_state_ == "ARMING") {
            if (counter_ < 110 || !has_odometry_) return;
            
            takeoff_position_ = {vehicle_odometry_.position[0], 
                                vehicle_odometry_.position[1], 
                                vehicle_odometry_.position[2]};
            current_waypoint_ = {takeoff_position_.x, takeoff_position_.y, TAKEOFF_HEIGHT};
            flight_state_ = "TAKEOFF";
            RCLCPP_INFO(get_logger(), "State: TAKEOFF to %.1fm", TAKEOFF_HEIGHT);
            
        } else if (flight_state_ == "TAKEOFF") {
            publish_position_setpoint(current_waypoint_.x, current_waypoint_.y, current_waypoint_.z);
            if (at_position(current_waypoint_.x, current_waypoint_.y, current_waypoint_.z)) {
                current_waypoint_.x += FORWARD_DISTANCE;
                flight_state_ = "FORWARD";
                RCLCPP_INFO(get_logger(), "State: FORWARD - Moving %.1fm", FORWARD_DISTANCE);
            }
            
        } else if (flight_state_ == "FORWARD") {
            publish_position_setpoint(current_waypoint_.x, current_waypoint_.y, current_waypoint_.z);
            if (at_position(current_waypoint_.x, current_waypoint_.y, current_waypoint_.z)) {
                hover_start_time_ = now();
                flight_state_ = "HOVER";
                RCLCPP_INFO(get_logger(), "State: HOVER for %.1fs", HOVER_TIME);
            }
            
        } else if (flight_state_ == "HOVER") {
            publish_position_setpoint(current_waypoint_.x, current_waypoint_.y, current_waypoint_.z);
            auto elapsed = (now() - hover_start_time_).seconds();
            if (elapsed >= HOVER_TIME) {
                current_waypoint_.z = 0.0f;
                flight_state_ = "LANDING";
                RCLCPP_INFO(get_logger(), "State: LANDING");
            }
            
        } else if (flight_state_ == "LANDING") {
            publish_position_setpoint(current_waypoint_.x, current_waypoint_.y, current_waypoint_.z);
            if (has_odometry_ && vehicle_odometry_.position[2] > -0.2f) {
                disarm();
                flight_state_ = "LANDED";
                RCLCPP_INFO(get_logger(), "State: LANDED - Mission Complete!");
            }
        }
    }

    void mapping_loop() {
        if (!has_scan_data_ || !has_odometry_) {
            RCLCPP_WARN(get_logger(), "Waiting for LiDAR and odometry data...");
            return;
        }

        if (flight_state_ == "FORWARD" && !mapping_begin_) {
            mapping_begin_ = true;
        }

        if (mapping_begin_) {
            mapping_iterations_++;
            
            RCLCPP_INFO(get_logger(), "Updating occupancy grid with %zu beams...", 
                       latest_scan_data_.ranges.size());
            
            float robot_theta = 0.0f;
            update_occupancy_grid(vehicle_odometry_.position[0], 
                                vehicle_odometry_.position[1], robot_theta);
            
            if (mapping_iterations_ >= 5) {
                mapping_timer_->cancel();
                RCLCPP_INFO(get_logger(), "Completed 5 mapping iterations");
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IntegratedDroneMappingNode>());
    rclcpp::shutdown();
    return 0;
}
