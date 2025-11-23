#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv); //Initialize ros2 communication
    auto node = std::make_shared<rclcpp::Node>("cpp_test"); // Initialize the node using auto type
    RCLCPP_INFO(node->get_logger(), "Hello world"); // logging the info into the terminal
    rclcpp::spin(node); // Keeping the node running until user presses ctrl + c
    rclcpp::shutdown();
    return 0;
}