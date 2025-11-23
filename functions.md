📦 Imports and Setup 
| Line   | Code |Explanation | 
|---|---|---|
|1. |import rclpy |Imports the core ROS 2 Python client library. This module provides all the fundamental functions for writing ROS 2 applications, such as initialization, spinning, and shutdown.|
| 2. |from rclpy.node import Node | Imports the Node class. All ROS 2 components that can publish, subscribe, or use services must inherit from this base class, which provides the core ROS 2 communication and timing utilities.|

🤖 The Node Class (MyNode)

This class defines your custom ROS 2 program. It encapsulates all the logic, state (like the counter), and ROS 2 utilities (like the timer).

| Line	| Code	| Explanation |
|----|----|----|
|4	| class MyNode(Node):	| Defines a new class named MyNode that inherits from rclpy.node.Node. This grants MyNode all the capabilities of a standard ROS 2 node. |
|5	| def __init__(self):	| The constructor for the MyNode class. This runs when an object of this class is created.|
|6	| super().__init__('py_test')	| Calls the constructor of the parent class (Node). This is a mandatory step that registers the node with the ROS 2 system, assigning it the unique name py_test.|
| 7	| self.get_logger().info('...')	| Logs an informational message to the ROS 2 system. self.get_logger() retrieves the logger associated with the node, and .info() prints the message to the console with the [INFO] tag.|
| 9	| self.create_timer(1.0,self.timer_callback)	| Creates and schedules a recurring timer. It tells the node to call the method self.timer_callback every 1.0 second. This is a non-blocking, periodic event.|
| 10	| self.count_=1	| Initializes a member variable. This variable is used to keep track of how many times the timer callback has executed, starting at 1.
| 12	| def timer_callback(self):	| Defines the callback function that is executed every time the timer expires (i.e., every 1.0 second). It takes no arguments other than self.| 
| 13	| string_text = "hello" + str(self.count_)	| Constructs a log message. It concatenates the string "hello" with the string representation of the current counter value.|
| 14	| self.get_logger().info(string_text)	| Logs the constructed message to the console.|
| 15	| self.count_= self.count_+1	| Increments the counter by 1, preparing for the next timer event. |

🚀 Execution Functions (main)

These functions handle the startup and shutdown of the ROS 2 environment.

| Line	| Code	| Explanation |
|---|---|---|
|18	| def main(args=None):	|The standard entry point function for a ROS 2 executable script. args=None allows command-line arguments to be passed.|
| 19	| rclpy.init(args=args)	| Initializes the ROS 2 client library. This must be called first in any ROS 2 program to set up the necessary underlying communication mechanisms.| 
| 20	| node= MyNode()	| Creates an instance of your custom MyNode class. This executes the __init__ method, which initializes the counter and starts the 1-second timer. |
| 21	| rclpy.spin(node)	| Blocks execution and starts the node's event loop. This is the core function where the node listens for and processes events (like the timer callbacks). It keeps the node alive until a shutdown signal is received (e.g., Ctrl+C or a program shutdown request).| 
| 22	| rclpy.shutdown()	| Shuts down the ROS 2 client library. This is called after rclpy.spin() returns and is necessary to free up any allocated resources.|
| 24	| if __name__=="__main__":	| Standard Python check that ensures the main() function is called only when the script is executed directly (not when it is imported as a module).| 
| 25	| main()	| Executes the primary entry point function.|