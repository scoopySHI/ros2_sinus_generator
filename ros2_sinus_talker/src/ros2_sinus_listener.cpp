
#include <memory>
#include <cmath>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
using std::placeholders::_1;

class SinusSubscriber : public rclcpp::Node
{
public:
  SinusSubscriber()
  : Node("Sinus_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "sinus_chatter", default_qos_profile, std::bind(&SinusSubscriber::topic_callback, this, _1));
    publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "sinus_after_chatter", default_qos_profile);
  }

  int rec_count_ = 0;

private:
  void topic_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    rec_count_++;
    //RCLCPP_INFO(this->get_logger(), "I heard %f", msg->data);
    int i;

    for (i = 0; i < 1000000; i++){
      int rnd = std::rand();
      double r = sqrt(static_cast<double>(rnd)) * sqrt(static_cast<double>(rnd));
      if(rint(r) != rnd)
      {
        std::cout << "sqrt error!\n";
      }
    }

    publisher_->publish(*msg);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SinusSubscriber>();
  rclcpp::spin(node);
  std::cout << "Received msg: " << node->rec_count_ << "\n";
  rclcpp::shutdown();
  return 0;
}
