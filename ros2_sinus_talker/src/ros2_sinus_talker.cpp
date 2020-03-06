#include <chrono>
#include <cstdio>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

void print_usage()
{
  printf("Usage for sinustalker app:\n");
  printf("sinustalker [-p phase] [-r rate] [-a amplitude] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-p phase : Specify the phase. \n");
  printf("-r rate : Specify the run cycle time (ms). \n");
  printf("-a amplitude : Specify the amplitude (ms). \n");
}

class Talker : public rclcpp::Node
{
public:
  explicit Talker(
    double phase, double amplitude,
    const std::chrono::nanoseconds cycle_time)
  : Node("talker_node"), count_(0.0),
    phase_(phase), amplitude_(amplitude),cycle_time_(cycle_time)
  {
    sinus_pub_ = this->create_publisher<std_msgs::msg::Float64>("sinus_chatter", default_qos_profile);
    func_time_pub_ = this->create_publisher<std_msgs::msg::Float64>("func_run_time", default_qos_profile);
    spin_time_pub_ = this->create_publisher<std_msgs::msg::Float64>("spin_run_time", default_qos_profile);
    cycle_time_pub_ = this->create_publisher<std_msgs::msg::Float64>("cycle_run_time", default_qos_profile);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(
      cycle_time_, std::bind(&Talker::timer_callback, this));
  }

private:

  void timer_callback()
  {
    auto spin_start = std::chrono::high_resolution_clock::now();

    auto sinus_msg = std::make_shared<std_msgs::msg::Float64>();
    auto func_time_msg = std::make_shared<std_msgs::msg::Float64>();
    auto spin_time_msg = std::make_shared<std_msgs::msg::Float64>();
    auto cycle_time_msg = std::make_shared<std_msgs::msg::Float64>();

    count_ += phase_;

    auto func_start = std::chrono::high_resolution_clock::now();
    sinus_msg->data = amplitude_ * cos(count_);
    auto func_over = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> diff = func_over - func_start;
    func_time_msg->data = diff.count();
    diff = spin_start - last_start_;
    last_start_ = spin_start;
    spin_time_msg->data = diff.count();

    sinus_pub_->publish(*sinus_msg);
    RCLCPP_INFO(this->get_logger(), "Sinus msg: %f", sinus_msg->data);
    func_time_pub_->publish(*func_time_msg);
    RCLCPP_INFO(this->get_logger(), "functime: %f", func_time_msg->data);
    spin_time_pub_->publish(*spin_time_msg);
    RCLCPP_INFO(this->get_logger(), "spintime: %f", spin_time_msg->data);

    auto cycle_over = std::chrono::high_resolution_clock::now();
    diff = cycle_over - spin_start;
    cycle_time_msg->data = diff.count();

    cycle_time_pub_->publish(*cycle_time_msg);
    RCLCPP_INFO(this->get_logger(), "cycletime: %f", cycle_time_msg->data);
  }

  double count_ = 0.0;
  double phase_ = 0.2;
  double amplitude_ = 0.5;
  std::chrono::nanoseconds cycle_time_;
  std::chrono::high_resolution_clock::time_point last_start_ = std::chrono::high_resolution_clock::now();
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sinus_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr func_time_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr spin_time_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cycle_time_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  int rate;
  double amp, phase;
  std::shared_ptr<Talker> nd_ptr{nullptr};

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  rclcpp::init(argc, argv);

  char * cli_option_phase = rcutils_cli_get_option(argv, &argv[argc], "-p");
  if (nullptr != cli_option_phase) {
    phase = std::stod(std::string(cli_option_phase));
    }
  char * cli_option_rate = rcutils_cli_get_option(argv, &argv[argc], "-r");
  if (nullptr != cli_option_rate) {
    rate = std::stoi(std::string(cli_option_rate));
    }
  char * cli_option_amp = rcutils_cli_get_option(argv, &argv[argc], "-a");
  if (nullptr != cli_option_amp) {
    amp = std::stod(std::string(cli_option_amp));
    }

  // Create a node.
  nd_ptr = std::make_shared<Talker>(phase, amp,
    std::chrono::milliseconds(rate));

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(nd_ptr);

  rclcpp::shutdown();
  return 0;
}
