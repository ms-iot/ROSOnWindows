# Getting Started with Moveit! and UR3
Getting Started with the Moveit! and UR3 running Windows.

## Prerequisite
This guide assumes you had hands-on experience on running ROSOnWindows from this document. (https://ms-iot.github.io/ROSOnWindows/GettingStarted/Setup.html)

### Moveit! and UR3 on Windows Installation
First, you can get started by installing moveit related packages from ROSOnWindows Chocolatey server.
```
(open a command prompt as admin)
> choco upgrade ros-melodic-moveit
```

Then, create a workspace, checkout and build the Universal Robot Driver for UR3.
```
(open a command prompt as admin)
> c:\opt\ros\melodic\x64\setup.bat
> mkdir c:\catkin_ws\src
> cd c:\catkin_ws\src
> git clone https://github.com/ms-iot/universal_robot -b init_windows
> git clone https://github.com/ms-iot/ur_modern_driver -b init_windows
> cd c:\catkin_ws
> catkin_make
> c:\catkin_ws\devel\setup.bat
```

Now you are good to go to run UR3 launch files. Before proceeding, make sure your UR3 controller is on and the network is connected to your dev box.

> Currently the simulation stack is not yet ported for ROS on Windows, so you might see build break on `ur_gazebo`. To workaround that, remove the `ur_gazebo` subfolder from `c:\catkin_ws\src\universal_robot` directory, and run `catkin_make` again.

### Running UR3 Launch Files
In this example, it requires three launch files to run: One is to run the UR3 driver stack for planning execution, one is to run the UR3 motion planning, and the other one is to run the visualization tool.

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
