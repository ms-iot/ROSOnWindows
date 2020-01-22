# MoveIt Tutorials on Windows
This guide is to show you how to prepare a workspace (for ROS on Windows) for your MoveIt tutorials. Find out more about MoveIt, visit [here](https://ros-planning.github.io/moveit_tutorials/index.html).

## MoveIt Binary Installation on Windows
First, download the ROS on Windows with MoveIt packages.
(open a command prompt as admin)

```no-highlight
choco upgrade ros-melodic-moveit -y
```

## Create Workspace for MoveIt Tutorials
Then, let's download the example code to your workspace.

(open a command prompt as admin)

```no-highlight
mkdir c:\moveit_ws\src
cd c:\moveit_ws\src
catkin_init_workspace
git clone https://github.com/ms-iot/moveit_tutorials -b init_windows
git clone https://github.com/ms-iot/panda_moveit_config -b init_windows
git clone https://github.com/frankaemika/franka_ros-release -b release/kinetic/franka_description
cd c:\moveit_ws
catkin_make
```

After it is built, source the catkin workspace.

```no-highlight
c:\moveit_ws\devel\setup.bat
```

## Getting Started with MoveIt Tutorials
Now you are ready to continue the journal on [MoveIt Tutorials](https://ros-planning.github.io/moveit_tutorials/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html).
