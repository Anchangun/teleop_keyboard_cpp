#include"teleop_keyboard/teleop_keyboard.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopKeyboard>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}