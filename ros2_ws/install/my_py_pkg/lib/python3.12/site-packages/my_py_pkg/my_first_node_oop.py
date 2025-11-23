import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('py_test')
        self.get_logger().info('Hello World this is node created using OOP')
        self.create_timer(1.0,self.timer_callback)
        self.count_=1

    def timer_callback(self):
        string_text = "hello" + str(self.count_)
        self.get_logger().info(string_text)
        self.count_= self.count_+1


def main(args=None):
    rclpy.init(args=args)
    node= MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()