#ifndef TELEOP_KEYBOARD_HPP_
#define TELEOP_KEYBOARD_HPP_
#include <memory>
#include <termios.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
/**
 * @brief Robot control class using ros2 as keyboard
 * @author changun516@gmail.com
 */
using namespace std::chrono_literals;
class TeleopKeyboard : public rclcpp::Node{
public:
  TeleopKeyboard();
  virtual ~TeleopKeyboard();
private:
  void start_display();
  void publish_command();
  void read_key();
  void print_vels();
  double check_linear_limit_vel(double vel);
  double check_angular_limit_vel(double vel);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  struct termios settings_;

  double target_linear_velocity_;
  double target_angular_velocity_;
  geometry_msgs::msg::Twist prev_twist_;

  char key;
  bool flag_key;
  int kbhit();
};


#endif