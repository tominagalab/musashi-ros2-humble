#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MaxonEposManager : public rclcpp::Node {
 public:
  MaxonEposManager() : Node("maxon_epos_manager") {
    _timer = this->create_wall_timer(
        250ms, std::bind(&MaxonEposManager::timer_callback, this));
  }

 private:
  void timer_callback() { RCLCPP_INFO(this->get_logger(), "callback"); }

  rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MaxonEposManager>());
  rclcpp::shutdown();
  return 0;
}