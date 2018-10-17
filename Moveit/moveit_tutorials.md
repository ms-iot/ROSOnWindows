# Moveit! Tutorials on Windows
This guide is to show you how to prepare a workspace (for ROS on Windows) for your Moveit! tutorials. Find out more about Moveit!, visit https://ros-planning.github.io/moveit_tutorials/index.html.

## Moveit! Binary Installation on Windows
First, download the ROS on Windows with Moveit! packages.
```
(open a command prompt as admin)
> choco upgrade ros-melodic-moveit -y
```

## Create Workspace for Moveit! Tutorials
Then, let's download the example code to your workspace.

```
(open a command prompt as admin)
> mkdir c:\moveit_ws\src
> cd c:\moveit_ws\src
> catkin_init_workspace
> git clone https://github.com/ms-iot/moveit_tutorials -b init_windows
> git clone https://github.com/ms-iot/panda_moveit_config -b init_windows
> git clone https://github.com/frankaemika/franka_ros-release -b release/kinetic/franka_description
> cd c:\moveit_ws
> catkin_make
```

After it is built, source the catkin workspace.

```
> c:\moveit_ws\devel\setup.bat
```

## Getting Started with Moveit! Tutorials
Now you are ready to continue the journal on [Moveit! Tutorials](https://ros-planning.github.io/moveit_tutorials/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html).
