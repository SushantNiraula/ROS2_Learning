import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")
        self.declare_parameter("robot_name", "Sushant_bot")
        self.publisher_=self.create_publisher(String,"robot_news", 10)
        self.timer_= self.create_timer(0.5,self.publish_news)
        self.robot_name_=self.get_parameter("robot_name")
        self.get_logger().info(f"Hi this is {self.robot_name_} ")
        self.count=0
    def publish_news(self):
        msg= String()
        self.count+=1
        msg.data = "I hope you are all well right !... " + str(self.count)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node=RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()