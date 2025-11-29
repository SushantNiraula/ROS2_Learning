#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::placeholders;

class NumberCounterNode: public rclcpp::Node{
    public:
        NumberCounterNode(): Node("number_counter"), counter_(0)
        {
            number_subscriber_= this->create_subscription<example_interfaces::msg::Int64>("number", 10, std::bind(&NumberCounterNode::numberCallback, this, _1));
            counter_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);

        }
    private:
        void numberCallback(const example_interfaces::msg::Int64::SharedPtr msg)
        {
            counter_ += msg->data;
            auto newMsg = example_interfaces::msg::Int64();
            newMsg.data = counter_;
            RCLCPP_INFO(this->get_logger(), "Got the number from number_publisher: %ld", msg->data);
            counter_publisher_->publish(newMsg);
        }
        int counter_;
        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr counter_publisher_;
        rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr number_subscriber_;
};
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}