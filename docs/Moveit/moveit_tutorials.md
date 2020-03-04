# MoveIt Tutorials on Windows
This guide is to show you how to prepare a workspace (for ROS on Windows) for your MoveIt tutorials. Find out more about MoveIt, visit [here](https://ros-planning.github.io/moveit_tutorials/index.html).

## Install ROS on Windows
Follow the ROS on Windows installation istructions. You may skip the Binary Package Installation step and install the ros-melodic-moveit binary instead of ros-melodic-desktop_full or ros-eloquent-desktop (as shown in the next section).
http://wiki.ros.org/Installation/Windows

## MoveIt Binary Installation on Windows
Download the ROS on Windows with MoveIt packages.
(open an elevated ROS Command Window as described in the installation instructions)

```no-highlight
mkdir c:\opt\chocolatey
set ChocolateyInstall=c:\opt\chocolatey
choco source add -n=ros-win -s="https://roswin.azurewebsites.net/api/v2" --priority=1
choco upgrade ros-melodic-moveit -y --execution-timeout=0
```

Close the window and open a new ROS Command Window. Make sure you have the most up to date packages:
```rosdep update
```

## Create Workspace for MoveIt Tutorials
Then, create a workspace and download the example code.

(open a command prompt as admin)

```no-highlight
mkdir c:\moveit_ws\src
cd c:\moveit_ws\src
catkin_init_workspace
git clone https://github.com/ros-planning/moveit_tutorials.git -b melodic-devel
git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel
git clone https://github.com/frankaemika/franka_ros-release -b release/kinetic/franka_description
cd c:\moveit_ws
catkin_make
```

After it is built, source the catkin workspace.

```no-highlight
c:\moveit_ws\devel\setup.bat
```

## Getting Started with MoveIt Tutorials
Now you are ready to continue to the MoveIt Quickstart in RViz section of the tutrials [MoveIt Tutorials](https://ros-planning.github.io/moveit_tutorials/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html).
