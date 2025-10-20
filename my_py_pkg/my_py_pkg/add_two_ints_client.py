import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.client= self.create_client(AddTwoInts, "add_two_ints")

    def call_add_to_ints(self, a , b):
        while not self.client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Add Two Ints server...")
        request= AddTwoInts.Request()
        request.a=a
        request.b=b

        future=self.client.call_async(request)
        future.add_done_callback(self.callback_call_add_two_ints)

    def callback_call_add_two_ints(self, future):
        response = future.result()
        self.get_logger().info("Got Response: " + str(response.sum))


    
def main(args=None):
    rclpy.init(args=args)
    node= AddTwoIntsClient()
    node.call_add_to_ints(2,7)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
