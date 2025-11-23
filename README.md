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