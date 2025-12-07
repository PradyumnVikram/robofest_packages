#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "map_generator.hpp"

class MineMapper : public rclcpp::Node {
public:
    MineMapper() : Node("mine_mapper") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera",
            10,
            std::bind(&MineMapper::listener_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Mine mapping node started");
    }

private:
    mutable MapGen map_obj;
    double* pos = (double* )calloc(2, sizeof(double));
    void listener_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8") -> image;
            cv::imshow("Camera", frame);
            map_obj.update_map(frame, pos);
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
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MineMapper>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
