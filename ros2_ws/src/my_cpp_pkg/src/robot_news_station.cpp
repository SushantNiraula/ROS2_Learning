#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>

using namespace std::chrono_literals;

class RobotNewsStation : public rclcpp::Node //This class inherits from rclcpp::Node
{
    public:
    RobotNewsStation() : Node("robot_news_station"), robot_name_("ROS2_Robot_Sushant")
    {
        publisher_= this->create_publisher<example_interfaces::msg::String>("robot_news",10);
        timer_ = this->create_wall_timer(0.5s, std::bind(&RobotNewsStation::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "Robot_News_Station node has been Started");
    }

    private:
    void publishNews(){
        auto msg= example_interfaces::msg::String();
        msg.data = std::string("Hello, this is ") + robot_name_ + std::string(" From the robot news station.");
        publisher_->publish(msg);
    }
        std::string robot_name_;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotNewsStation>());
    rclcpp::shutdown();
    return 0;
}