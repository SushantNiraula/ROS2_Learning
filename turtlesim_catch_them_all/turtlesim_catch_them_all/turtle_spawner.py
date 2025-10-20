import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from functools import partial
import random
import math
from my_robot_interfaces.msg import Turtle, TurtleArray

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.turtle_name_prefix_ = "turtle"
        self.alive_turtles_=[]
        self.turtle_counter_= 2
        self.spawn_client_=self.create_client(Spawn, "/spawn")
        self.spawn_turtle_timer_= self.create_timer(2.0,self.spawn_new_turtle )
        self.alive_turtle_publisher_= self.create_publisher(TurtleArray, "alive_turtles", 10)

    def publish_alive_turtles(self):
        msg=TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtle_publisher_.publish(msg)

    def spawn_new_turtle(self):
        name = self.turtle_name_prefix_ + str(self.turtle_counter_)
        self.turtle_counter_+=1
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0 , 2* math.pi)
        self.call_spawn_service(name, x, y, theta)

    def call_spawn_service(self, turtle_name, x, y, theta):
        while not self.spawn_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Spawn Service ...")
        request= Spawn.Request()
        request.x=x
        request.y= y
        request.theta = theta
        request.name= turtle_name
        future= self.spawn_client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawn_service, request=request))

    def callback_call_spawn_service(self, future, request):
        response: Spawn.Response = future.result()
        if response.name != "":
            self.get_logger().info("New alive turtle: " + response.name)
            new_turtle = Turtle()
            new_turtle.name= response.name
            new_turtle.x= request.x
            new_turtle.y= request.y
            new_turtle.theta = request.theta
            self.alive_turtles_.append(new_turtle)
            self.publish_alive_turtles()
        
    
def main(args=None):
    rclpy.init(args=args)
    node= TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()