# Getting Started with Moveit! and UR3 on Windows
Getting Started with the Moveit! and UR3 on Windows. If you are new to Moveit! or you don't have a real robot, check out [Moveit! Tutorials on Windows](moveit_tutorials.md).

## Prerequisite
* This guide assumes you had hands-on experience on running ROSOnWindows from this document. (https://ms-iot.github.io/ROSOnWindows/GettingStarted/Setup.html)

* This guide requires you have an Universal Robot UR3 with a software version (3.x)

## Moveit! and UR3 on Windows Installation
First, you can get started by installing moveit related packages from ROSOnWindows Chocolatey server.
```
(open a command prompt as admin)
> choco upgrade ros-melodic-moveit -y
```

Then, create a workspace, checkout and build the Universal Robot Driver for UR3.
```
(open a command prompt as admin)
> c:\opt\ros\melodic\x64\setup.bat
> mkdir c:\catkin_ws\src
> cd c:\catkin_ws\src
> git clone https://github.com/ms-iot/universal_robot -b init_windows
> git clone https://github.com/ms-iot/ur_modern_driver -b init_windows
> echo > universal_robot\ur_gazebo\CATKIN_IGNORE
> cd c:\catkin_ws
> catkin_make
> c:\catkin_ws\devel\setup.bat
```

> As of October 2018, Gazebo has not been ported. We create `universal_robot\ur_gazebo\CATKIN_IGNORE` to asks catkin disabling Gazebo related package.

Now you are almost good to go to run UR3 launch files. Before proceeding, make sure your UR3 controller is on and the network is connected to your dev box.

## Running UR3 Launch Files
Now let's run it! In this example, it requires three launch files to run: One is to run the UR3 driver stack for planning execution, one is to run the UR3 motion planning, and the other one is to run the visualization tool.

Let's start the UR3 driver stack:
```
(open a command prompt as admin)
> c:\opt\ros\melodic\x64\setup.bat
> c:\catkin_ws\devel\setup.bat
> roslaunch ur_modern_driver ur3_bringup.launch robot_ip:=IP_OF_THE_ROBOT use_lowbandwidth_trajectory_follower:=true
```

Second, run the UR3 motion planning:
```
(open a command prompt as admin)
> c:\opt\ros\melodic\x64\setup.bat
> c:\catkin_ws\devel\setup.bat
> roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch
```

Finally, run the visualization tool:
```
(open a command prompt as admin)
> c:\opt\ros\melodic\x64\setup.bat
> c:\catkin_ws\devel\setup.bat
> roslaunch ur3_moveit_config moveit_rviz.launch config:=true
```

> Known Issue: When you don't see all panels displayed in RViz, try to enter full screen mode and exit (F11) to refresh the windows rendering.

Now you are ready to move the robot arm in the visualization tool and start planning and see your arm moving in action!

## Troubleshoot
1. The arm fails to move and `Invalid Trajectory: start point deviates from current robot state more than ...` shows in motion planning console window. It is likely the default value of [allowed_start_tolerance](http://moveit.ros.org/moveit!/ros/2017/01/03/firstIndigoRelease.html) being too small for you. Before figuring out a reasonable value,  edit `ur3_moveit_config/launch/move_group.launch` and adding `allowed_start_tolerance` can help:
```
  <node name="move_group" launch-prefix="$(arg launch_prefix)" pkg="moveit_ros_move_group" type="move_group" respawn="false" output="screen" args="$(arg command_args)">
    ...
    <param name="trajectory_execution/allowed_start_tolerance" value="0.0"/> <!-- default 0.01, disable 0.0 -->
  </node>
```

## More Information
* [ur_modern_driver](https://github.com/seanyen-msft/rosonWindows)
* [Moveit! Tutorials](https://ros-planning.github.io/moveit_tutorials/)
