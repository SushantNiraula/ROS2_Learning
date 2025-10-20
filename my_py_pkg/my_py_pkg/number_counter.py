import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int16

class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.subscriber_=self.create_subscription(Int16, "number",self.callback_number,10)
        self.get_logger().info("number Counter has started !")
        self.publisher_=self.create_publisher(Int16, "number_count",10)
        self.count=0

    def callback_number(self, msg: Int16):
        self.count+=msg.data
        new_msg = Int16()
        new_msg.data= self.count
        self.publisher_.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node= NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()