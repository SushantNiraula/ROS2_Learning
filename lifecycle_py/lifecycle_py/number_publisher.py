import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from example_interfaces.msg import Int64

class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("number_publisher")
        self.get_logger().info("In Constructor")
        self.number_=1
        self.publish_frequency_= 1.0
        self.number_publisher_=None
        self.number_timer_= None
    
    ## Create ROS2 Communications, Connect to HW
    def on_configure(self, prev_state: LifecycleState) -> TransitionCallbackReturn:
        ## Create publisher / subscriber i.e create ROS2 communication and hardwares
        self.get_logger().info("In on_configure")
        self.number_publisher_=self.create_lifecycle_publisher(Int64, "number", 10)
        self.number_timer_=self.create_timer(1.0/self.publish_frequency_, self.publish_number)
        return TransitionCallbackReturn.SUCCESS

    # Activate/ Enable HW
    def on_activate(self, previous_state:LifecycleState):
        self.get_logger().info("IN on active")
        return super().on_activate(previous_state)

    # Deactivate/Diable HW
    def on_deactivate(self, prevous_state:LifecycleState):
        self.get_logger().info("In on_deactivate")
        return super().on_deactivate(prevous_state)

    ## Destroy ROS2 Communications, and disconnect from Hardware
    def on_cleanup(self, prev_state: LifecycleState):
        self.get_logger().info("In on_cleanup")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, prev_state:LifecycleState):
        self.get_logger().info("IN on_shutdown")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS

    ## PROCESSES ERROR , BUT PROBLEM IS WE DON'T KNOW WHAT CAUSED THE ERROR. so BETTER IS TO USE TRY 
    # EXCEPT EXCEPTION METHOD TO FIX ON THE SAME STATE.
    def on_error(self, prev_state:LifecycleState):
        self.get_logger().info("In on_error.")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)

        ## Do some checks, if okay then return SUCCESS , if not then FAILURE.
        return TransitionCallbackReturn.SUCCESS

    def publish_number(self):
        msg=Int64()
        msg.data=self.number_
        self.number_publisher_.publish(msg)
        self.number_+=1
    
def main(args=None):
    rclpy.init(args=args)
    node=NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()