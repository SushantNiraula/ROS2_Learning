import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class SmartphoneNode(Node):
    def __init__(self):
        super().__init__('smartphone')
        self.subscriber_=self.create_subscription(String,'robot_news',self.news_received,10)
        
    def news_received(self, msg: String):
        news_data= msg.data

        self.get_logger().info(news_data)

def main(args=None):
    rclpy.init(args=args)
    node= SmartphoneNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()