import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node= Node("my_test")
    ## Our node my_test is inside this python file.
    node.get_logger().info('Hello, World this is my first node.')
    ## To keep it alive and stop it when we want
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()