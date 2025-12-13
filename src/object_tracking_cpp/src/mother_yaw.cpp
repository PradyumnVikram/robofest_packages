#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"


using std::placeholders::_1;

class MothernodeYaw : public rclcpp::Node
{
  public:
    MothernodeYaw()
    : Node("MothernodeYaw")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/waist_angle", 10, std::bind(&MothernodeYaw::yaw_callback, this, _1));
    }

  private:
    void yaw_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%.2f'", msg->data[0]);
    }
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MothernodeYaw>());
  rclcpp::shutdown();
  return 0;
}