# ROS2 Basics
1. Starting with ROS2 installation in ubuntu desktop.

Use this url <a hred="https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html">Guide/doc to install ros2 Jazzy into your ubuntu laptop</a>

After Installation check if everything is working well. So see it just run simple demo_nodes.
>> ros2 run demo_nodes_cpp talker

it is a simple publisher node that counts every second.

>> ros2 run demo_nodes_cpp listener

it is a simple subscriber that subscribes to the talker node.

2. Create and Setup ROS2 Workspace

A ROS 2 workspace is a directory that serves as the central location for organizing and developing robot software, containing all related packages, code, data files, and configuration scripts.# ROS2_Learning

To create a ROS2 Workspace we follow series of steps:

    >> cd
    >> mkdir ros2_ws
    >> cd ros2_ws/
    >> mkdir src
    >> colcon build 
    
Then to acess our workspace functionaliy we need to source into bash as well. 
    
    >> gedit .bashrc

Then add the block of code at the end of bashrc

    >>source ~/Coding/ROS2_Learning/ROS2_Basics/ros2_ws/install/setup.bash

3. Create a Python Package for ROS2 inside our ROS2_workspace i.e ros2_ws
    
    To create a python package we use the following code. Always remember that package is created inside the src of the workspace.

    Go into your  workspace (For me which is)
    >> cd /Coding/ROS2_Learning/ROS2_Basics/ros2_ws/src/

    Then use the code


    >> ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
    
    Inside our newly created my_py_pkg we have folder structure as:
        -my_py_pkg 
            -__init__.py
        -package.xml  
        -resource  
        -setup.cfg  
        -setup.py  
        -test

    The folder my_py_pkg inside the package my_py_pkg is where create our nodes for our package.

    Before running and using the package we must build it:

    >> colcon build from workspace

    i.e we should be in /ros2_ws

    or we have other way of doing it too:

    >>colcon build --packages-select my_py_pkg

    This only build python package inside our workspace. this is selective building.
4. Create a C++ Package for ROS2 inside our ROS2_workspace i.e ros2_ws
    
    Again first thing to do is go to ros2_ws/src/ , THis is where we will create our CPP Pkg

    >> ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp

    after creating the python/cpp package to start working we go inside the package eiter my_py_pkg or my_cpp_pkg and then start working inside to solve our problem. This is an important way to work. 
    >>. code
5. Then after working and making code ready we must then build our package. <warning ><i>Always be sure to build the packages from ros2_ws/ directory not anywhere else.</i></warning>
6. Writing my first node i.e py_node and cpp_node . Just a simple node:
    goto src/my_py_pkg/my_py_pkg and then create a newfile:
    
    >>touch my_first_node.py

    then also make the python file executable. 

    >>chmod +x my_first_node.py

    Then get into writing code into your file my_first node. Here we will create one node.

    we can run the node manually using >> python3 my_first_node

    but in real world we don't do it this way. Instead we call from the package:

    to do it:
    1. Go to setup.py and make some changes as:
        entry_points={
        'console_scripts': [
        ],
    },
    2. we make it as:
        entry_points={
        'console_scripts': [
            "py_node = my_py_pkg.my_first_node:main",
        ],
    }
7. To run your node after writing go to workspac i.e ros2_ws and:
    >> colcon build --packages-select my_py_pkg
8. Then source the workspace using 
    >> source ~/.bashrc

    and run the node using:

    >> ros2 run my_py_pkg py_node
    
    where py_node is the name of node you created i.e name in setup.py in actual file we have name of node as my_test. So keep that in mind.
9. We don't want our code to be like that, we want to use industry standard and build node with OOP. Done in my_first_node_oop.py

    Let's see series of steps necessary to start working on a node in ros2:

        a. cd my_py_pkg/my_py_pkg
        
        b. touch my_first_node_oop.py

        c. chmod +x my_first_node_oop.py   // This makes the python file executable which is essential for running the file using ros2 run command as we previously did.

        d. Then get into coding using oop the node. Which you can see in file my_first_node_oop.py and add it to setup.py using:

            entry_points={
            'console_scripts': [
                "py_node = my_py_pkg.my_first_node:main",
                "py_node_oop = my_py_pkg.my_first_node_oop:main", //this line is added
            ],
            },

        e. After writing the code then build and source the package using :

            >> cd ../../..

            >> colcon build --packages-select my_py_pkg

            >> source ~/.bashrc

        f. Run the node using:

            >> ros2 run my_py_pkg py_node_oop

10. Then learning to write node in cpp.
    Now we must do it because industry demand ros2 codes in cpp as they are fast t run and very memory efficient. 

        a. go to ros2_ws/src/my_cpp_pkg/src
        b. touch my_first_node.cp to create a cpp file named as my_first_node
        c. Write the code in the my_first_node.cpp
        d. then put into the CMakeLists.txt to make it an executable and mention its dependencies using: 
            add_executable(cpp_node src/my_first_node.cpp)
            ament_target_dependencies(cpp_node rclcpp)

            install(
                TARGETS
                cpp_node
                DESTINATION lib/${PROJECT_NAME}
            )
        e. Then build, source and run the node.

## OOP Template for Nodes in python and CPP

    ### Python
        import rclpy
        from rclpy.node import Node

        class MyNode(Node):
            def __init__(self):
                super().__init__('py_test') # Modify the name according to what you want.

        def main(args=None):
            rclpy.init(args=args)
            node= MyNode()
            rclpy.spin(node)
            rclpy.shutdown()

        if __name__=="__main__":
            main()
    

    ### CPP
        #include "rclcpp/rclcpp.hpp"

        class MyNode: public rclcpp::Node //Modify Name
        {
            public:
                MyNode(): Node("node_name") // Modify name
                {
                    
                    }
            private:
        
        };

        int main(int argc, char **argv){
            rclcpp::init(argc, argv);
            auto node = std::make_shared<MyNode>();
            rclcpp::spin(node);
            rclcpp::shutdown();
            return 0;
        }

-----
# Introduction to ROS2 Tools we need to use :

### Introspecting your nodes with ros2 cli(Command Line Interference)

1. Check if you have correctly sourced environment. To check use : >> cat .bashrc

    ! Make sure you have sources ros2 and your workspace in bashrc which will automatically source your environments whenever you launch the terminal.

2. >> ros2 !TAB !TAB 
        to check all the commands we have with ros2 
        like run, action , launch, node, pkg and many.
3. ros2 run demo_nodes_cpp talker 
    ros2 run demo_nodes_cpp listener
4. ros2 node list # to get the list of all the nodes currently running in your ros2 environment
5. ros2 node info # to interospect the node and we cannot have two nodes with same name

We discussed that we cannot have two nodes with same name. but how to open two nodes of same function but with differnet name:

6. Rename the node at runtime : To ensure the nodes start with different name than from defualt. For example we have a node to read temperature using temperature sensor. But now we have 10 different sensors and we want all the data then how to do it. We cannot make 10 different nodes with different name. There comes renaming node at runtime. 
    ![Two nodes with same name
    ](<Screenshot from 2025-11-23 13-38-23.png>)

to rename:

    >> ros2 run my_py_pkg py_node --ros-args -r __node:=abc1
    >> ros2 run my_py_pkg py_node --ros-args --remap __node:=abc2
    Both does same thing

7. Colcon: 

Definition: Colcon stands for COmmand Line CONstructor. It is a command-line tool designed to build sets of software packages.s

Purpose: It allows developers to build entire ROS 2 workspaces, which may contain dozens or even hundreds of packages, in a structured and efficient way.

Parallel Builds: By default, colcon builds packages in parallel to speed up compilation. For resource-limited systems (like Raspberry Pi), you can use --executor sequential to build packages one at a time.

Workspace Concept: A colcon workspace is like a warehouse containing multiple ROS 2 projects. You can build, test, and run them together.

Symlink Install: Colcon supports --symlink-install, which makes development faster by avoiding repeated copying of files during builds.Once you build with symlink install we can continue to develop the packages without building again. It will build automatically.(For development phase we use Symlink Install, in production we don't do it.{only for python packages})

⚙️ Common Colcon Commands
* colcon build → Builds all packages in the workspace.

* colcon test → Runs tests for packages.

* colcon test-result → Shows results of the tests.

* colcon list → Lists all packages in the workspace.

* colcon clean (via plugin) → Cleans build artifacts.

8. RQT and RQTGraph

RQT is a graphical tool suite in ROS 2 that provides plugins for visualizing, debugging, and interacting with your robot system. RQTGraph is one of its plugins, specifically used to visualize the ROS computation graph (nodes and topics) in a clear, interactive diagram.

🖥️ What is RQT?
Definition: RQT is a Qt-based framework in ROS that offers a collection of GUI tools (plugins) for developers.

Purpose: It makes working with ROS more user-friendly by providing graphical interfaces for tasks that are otherwise done via command line.

Capabilities:

* Visualize topics, parameters, and node connections.

* Plot data in real time (e.g., sensor readings).

* Monitor logs and diagnostics.

* Manage parameters dynamically.

Extensible: Developers can write their own plugins to extend RQT for custom needs

🔗 What is RQTGraph?
Definition: RQTGraph is a plugin within RQT that shows the ROS graph—the network of nodes and topics in your system.

Visualization:

* Displays all active nodes.

* Shows publishers and subscribers.

* Illustrates how topics connect nodes together.

Use Cases:

* Debugging communication issues (e.g., checking if a node is publishing correctly).

* Understanding system architecture at a glance.

* Teaching and learning ROS concepts visually.

⚙️ Example Workflow
* Start a ROS 2 system (e.g., turtlesim).

* Run ros2 run rqt_gui rqt_gui to open RQT.

* Select Plugins → Visualization → rqt_graph.

* You’ll see a diagram of nodes (/turtlesim, /teleop_turtle) and topics (/turtle1/cmd_vel, /turtle1/pose).

Let's visualize and apply to what we have learned so far.

>> ros2 run turtlesim turtlesim_node 

Here we are opening turtlesim gui node for now.

>> ros2 run turtlesim turtle_teleop_key

This node will read from the keyboard to move the turtle.

Let's open the turtle sim using our custom name for the node or turtle.

>> ros2 run turtlesim turtlesim_node --ros-args -r node:=my_turtle

### Topics in ROS2.
📡 What is a Topic?
A topic is a named bus over which nodes exchange messages.

Nodes can publish messages to a topic, and other nodes can subscribe to that topic to receive those messages.

Topics are anonymous: publishers and subscribers don’t need to know about each other, only the topic name and message type.

🔑 Characteristics of Topics in ROS 2
* Message-based: Each topic carries messages of a specific type (e.g., std_msgs/msg/String, geometry_msgs/msg/Twist).

* Many-to-many: Multiple publishers can send to the same topic, and multiple subscribers can listen to it.

* Decoupled communication: Nodes don’t directly talk to each other; they only interact via topics.

* Asynchronous: Messages are delivered whenever they are published, not in a request-response manner.

* DDS-backed: ROS 2 uses the Data Distribution Service (DDS) middleware, which provides reliability, QoS (Quality of Service), and discovery for topics.

⚙️ Example
* Suppose you have a robot with a camera and a navigation system:

* The camera node publishes images on /camera/image_raw.

* The vision node subscribes to /camera/image_raw to process images.

* The navigation node publishes velocity commands on /cmd_vel.

* The motor controller node subscribes to /cmd_vel to move the robot.

\🛠️ Useful Commands
* ros2 topic list → Lists all active topics.

* ros2 topic echo /topic_name → Prints messages being published on a topic.

* ros2 topic info /topic_name → Shows publishers, subscribers, and message type.

* ros2 topic pub /topic_name msg_type "data" → Publishes a message manually.

### 📡 Publishers in ROS 2
A publisher is a node that sends messages on a topic.

It creates a communication endpoint and pushes data (like sensor readings, commands, or status updates) into the ROS 2 system.

Example:

a. A camera node publishes images on the topic /camera/image_raw.

b. A laser scanner node publishes distance measurements on /scan.

### 📥 Subscribers in ROS 2
A subscriber is a node that receives messages from a topic.

It listens to the topic and processes incoming data.

Example:

* A vision node subscribes to /camera/image_raw to process images.

* A navigation node subscribes to /scan to build a map of the environment.

### 🔗 Relationship Between Publishers and Subscribers
Publishers and subscribers are decoupled: they don’t know about each other directly, only the topic name and message type.

Multiple publishers can publish to the same topic, and multiple subscribers can listen to it.

ROS 2 middleware (DDS) handles discovery, delivery, and Quality of Service (QoS).

⚙️ Example Workflow
Imagine a robot:

* Publisher: /teleop_node publishes velocity commands (geometry_msgs/msg/Twist) on /cmd_vel.

* Subscriber: /motor_controller subscribes to /cmd_vel and moves the robot accordingly.

🛠️ Useful Commands
* ros2 topic list → See all topics.

* ros2 topic info /cmd_vel → Shows which nodes are publishing and subscribing.

* ros2 topic echo /cmd_vel → Displays messages being published.

✅ In short:

Publisher = sender of data on a topic

Subscriber = receiver of data from a topic Together, they form the backbone of ROS 2’s publish–subscribe communication model.

### To understand publisher subscriber we use robot_news_station and reciever files and nodes to be publisher and subscriber's

#### Python
we use use a example of robot_news_station which will be the pubisher and publish a msg (topic).

We will be using example_interfaces.msg -> String type of msg as our topic.

!! Keep in mind that if we use this type then we need to update our dependencies in `package.xml`
    <depend>example_interfaces</depend>

        import rclpy
        from rclpy.node import Node
        from example_interfaces.msg import String

        class RobotNewsStation(Node):
            def __init_(self):
                super().__init__('robot_news_station')
                self.publisher_= self.create_publisher(String, 'robot_news', 10)
                self.timer_= self.create_timer(2, self.publish_news)
            def publish_news(self):
                msg = String()
                msg.data= 'Sushant is learning ROS2 like a pro!'
                self.publisher_.publish(msg)


        def main(args=None):
            rclpy.init(args=args)
            node= RobotNewsStation()
            rclpy.spin(node)
            rclpy.shutdown()

        if __name__=="__main__":
            main()

After writing the code create executable in `setup.py` using:
    "robot_news_station = my_py_pkg.robot_news_station:main"
    
in entry_points of `setup.py`

Then colcon build the package and source and just run like you do :

>> ros2 run my_py_pkg robot_news_station

The topic 'robot_news' is published by the subscriber but currently we cannot see it.

Now let's learn to build a subscriber who will subscribe to topic by the publisher to receive the msg in the topic.

* create a python file smartphone.py as a receiver/subscriber.
* Then code it with code where we create a subscriber:

        import rclpy
        from rclpy.node import Node
        from example_interfaces.msg import String

        class SmartphoneNode(Node):
            def __init__(self):
                super().__init__('smartphone')
                self.subscriber_=self.create_subscription(String,'robot_news',self.news_received,10)
            
            def news_received(self, msg: String):
                news_data= msg.data
                self.get_logger().info(news_data)

        def main(args=None):
            rclpy.init(args=args)
            node= SmartphoneNode()
            rclpy.spin(node)
            rclpy.shutdown()

        if __name__=='__main__':
            main()


* then build, source and run the subscriber and publisher to publish and receive msg

### Publisher and Subscriber in CPP.

1. Start with going into the src of my_cpp_pkg from home.
    >> cd ros2_ws/src/my_cpp_pkg/src
2. touch robot_news_station.cpp for our cpp publisher node creation.
3. Then write code in the robot_news_station.cpp.
    We will need `example_interfacs/msg/string.hpp` so we need to include in package.xml 

    <depend>example_interfaces</depend>

    Then continue coding in robot_news_station.cpp


4. Then make changes in `CMakeLists.txt` as:
    add these lines:

            add_executable(robot_news_station src/robot_news_station.cpp)
            ament_target_dependencies(robot_news_station rclcpp example_interfaces)

            install(
                TARGETS
                robot_news_station
                DESTINATION lib/${PROJECT_NAME}
            )

5. Build, source and then run the code as we did. TO test we can see:
    >> ros2 topic list
    
    o/p:

        sushant-ros@sushant-ros:~$ ros2 topic list
        /parameter_events
        /robot_news
        /rosout

    then:

    >> ros2 topic echo /robot_news

    This will help us see the topic being published. We can also use our subscriber written in python to recieve the msg from spublisher written in cpp.

    >> ros2 run my_py_pkg smartphone

    Will work as ros2 is language independent as much as we write correct topic name.

6. Write the subscriber in cpp as similar to publisher the steps only the code for subscriber gets different.