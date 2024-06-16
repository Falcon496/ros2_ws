#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
//change
//#include "std_msgs/msg/string.hpp"
#include "tutorial_interfaces/msg/num.hpp"
using std::placeholders::_1;
class MinimalSubscriber : public rclcpp::Node
{
public:
 MinimalSubscriber()
 : Node("minimal_subscriber")
 {
    // change
    //subscription_ = this->create_subscription<std_msgs::msg::String>(
    subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>(
     "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
 }
private:
 // change
 //void topic_callback(const std_msgs::msg::String & msg) const
 //{
 // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
 //}
 //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
 void topic_callback(const tutorial_interfaces::msg::Num & msg) const
 {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.num << "'");
 }
 rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_;
};
int main(int argc, char * argv[])
{
 rclcpp::init(argc, argv);
 rclcpp::spin(std::make_shared<MinimalSubscriber>());
 rclcpp::shutdown();
 return 0;
}