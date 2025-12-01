import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client=self.create_client(AddTwoInts, 'add_two_ints')


    def call_add_two_ints(self, a, b):
        while not self.client.wait_for_service(1.0): ## SService available xaina vaney not False == True hunxa so parkhirakhxa
            self.get_logger().warn("waiting for Add Two Ints Server ...")
            
        request = AddTwoInts.Request()
        request.a = a
        request.b= b
        future=self.client.call_async(request)
        future.add_done_callback(self.callback_call_add_two_ints) # This is executed when we get response from the server
    def callback_call_add_two_ints(self, future):
        response = future.result()
        self.get_logger().info("Got response :" + str(response.sum))
def main(args=None):
    rclpy.init(args=args)
    node= AddTwoInts()
    node.call_add_two_ints(2,7)
    rclpy.spin()
    rclpy.shutdown()

if __name__=="__main__":
    main()