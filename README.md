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