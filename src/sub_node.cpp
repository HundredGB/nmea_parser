#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "nmea_parser/msg/gpsx.hpp"

using namespace std::chrono_literals;


using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
  Subscriber()
  : Node("gps_subscriber")
  {
    subscription_ = this->create_subscription<nmea_parser::msg::Gpsx>(
      "gpsx", 10, std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const nmea_parser::msg::Gpsx & message) const
  {
    RCLCPP_INFO(this->get_logger(), "date recieved"); //RCLCPP_INFO 와 std::cout은 무언가를 출력한다는데 있어서 동일하지만 RCLCPP_INFO는 ROS언어이고 std::cout은 c++언어라는 차이점이 있다.
    std::cout << message.utc_time << std::to_string(message.longitude) << std::to_string(message.latitude) << std::to_string(message.altitude) << std::to_string(message.satellites) << std::to_string(message.mode_indicator) << std::to_string(message.separation) << std::to_string(message.dilution) << std::endl;
  }
  rclcpp::Subscription<nmea_parser::msg::Gpsx>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}
