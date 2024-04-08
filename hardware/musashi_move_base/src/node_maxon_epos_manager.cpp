#include "rclcpp/rclcpp.hpp"

class maxon_epos_manager: public rclcpp::Node
{
  public:
    maxon_epos_manager() : Node("maxon_epos_manager") {
    }

  private:
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<maxon_epos_manager>());
  rclcpp::shutdown();
  return 0;
}