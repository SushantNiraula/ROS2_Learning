import rclpy 
from rclpy.node import Node
from my_robot_interfaces.msg import LedStateArray

class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel")
        self.led_states_=[0,0,0]
        self.led_states_pub_= self.create_publisher(LedStateArray, "led_panel_state", 10)
        self.led_state_timer_= self.create_timer(5.0, self.publish_led_state)
        self.get_logger().info("Led Pannel Node has been Started.")
    def publish_led_state(self):
        msg= LedStateArray()
        msg.led_states= self.led_states_
        self.led_states_pub_.publish(msg)
        
        

def main(args=None):
    rclpy.init(args=args)
    node= LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()