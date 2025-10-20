# import rclpy # Ros 2 Library for python
# from rclpy.node import Node
# def main(args= None):
#     rclpy.init(args=args) # To start Node
#     node= Node("py_test")
#     node.get_logger().info("Hello World")
#     rclpy.spin(node) # To keep node alive until pressed ctrl + C
#     rclpy.shutdown()
# if __name__=="__main__":
#     main()

## Improve the code with OOP
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("py_test")
        self.get_logger().info("Hello, World !")
        self.create_timer(1.0,self.timer_callback)
        self.time=0
    def timer_callback(self):

        self.get_logger().info(f"Hello {self.time}")
        self.time= self.time + 1
    
def main(args=None):
    rclpy.init(args=args)
    node=MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
