#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp> 
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "map_generator.hpp"
#include <Eigen/Geometry>

class MineMapper : public rclcpp::Node {
public:
    MineMapper() : Node("mine_mapper") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera",
            10,
            std::bind(&MineMapper::listener_callback, this, std::placeholders::_1));

        auto qos = rclcpp::SensorDataQoS();
        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            std::bind(&MineMapper::odom_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Mine mapping node started");
    }

private:
    MapGen map_obj;
    bool pos_valid_ = false;
    DronePose pos_quat;

    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        pos_quat.x = msg->position[0]; 
        pos_quat.y = msg->position[1]; 
        pos_quat.z = msg->position[2];
        pos_quat.q_w = msg->q[0];
        pos_quat.q_x = msg->q[1];
        pos_quat.q_y = msg->q[2];
        pos_quat.q_z = msg->q[3];
        
        RCLCPP_INFO_ONCE(this->get_logger(), "Odometry received: %.3f, %.3f", pos_quat.x, pos_quat.y);
        pos_valid_ = true;
    }

    void listener_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!pos_valid_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                            "Waiting for odometry (pos=%.3f,%.3f)", pos_quat.x, pos_quat.y);
            return;  // Skip until odometry arrives
        }
        try
        {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8") -> image;
            cv::imshow("Camera", frame);
            map_obj.update_map(frame, pos_quat);
            for(auto mine : map_obj.mine_locations){
                RCLCPP_INFO(this->get_logger(), "Mine at: %f, %f", mine.x, mine.y);
            }
            cv::waitKey(1);
        }
        catch (cv_bridge::Exception& e){
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MineMapper>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
