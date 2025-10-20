import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int16


class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.declare_parameter("number",2)
        self.declare_parameter("timer_period", 1.0)
        self.number_=self.get_parameter("number").value
        self.time_=self.get_parameter("timer_period").value
        self.publisher_=self.create_publisher(Int16,"number",10)
        self.timer=self.create_timer(self.time_,self.number_publish)
        self.get_logger().info("Number Publisher node has begun")
        self.count=0
    def number_publish(self):
        number= Int16()
        number.data=self.number_
        self.publisher_.publish(number)
        

def main(args=None):
    rclpy.init(args=args)
    node= NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()