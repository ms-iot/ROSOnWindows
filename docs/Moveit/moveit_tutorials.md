# MoveIt Tutorials on Windows
This guide is to show you how to prepare a workspace for MoveIt tutorials. Find out more about MoveIt, visit [here](https://moveit.ros.org/).

## MoveIt Binary Installation on Windows
Download the ROS on Windows with MoveIt packages.
(open an elevated ROS Command Window as described in the installation instructions)

=== "Noetic"

    ```bat
    mkdir c:\opt\chocolatey
    set ChocolateyInstall=c:\opt\chocolatey
    choco source add -n=ros-win -s="https://aka.ms/ros/public" --priority=1
    choco upgrade ros-noetic-desktop_full -y --execution-timeout=0
    ```

=== "Melodic"

    ```bat
    mkdir c:\opt\chocolatey
    set ChocolateyInstall=c:\opt\chocolatey
    choco source add -n=ros-win -s="https://aka.ms/ros/public" --priority=1
    choco upgrade ros-melodic-moveit -y --execution-timeout=0
    ```

## Create Workspace for MoveIt Tutorials
Then, create a workspace and download the example code.

(open a command prompt as admin)

=== "Noetic"

    ```bat
    :: activate ROS environment
    c:\opt\ros\noetic\x64\setup.bat

    :: checkout MoveIt tutorial packages
    mkdir c:\moveit_ws\src
    cd c:\moveit_ws\src

    git clone https://github.com/ros-planning/moveit_tutorials.git -b master
    git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel
    git clone https://github.com/frankaemika/franka_ros-release -b release/kinetic/franka_description

    :: build packages
    cd c:\moveit_ws
    catkin_make
    ```

=== "Melodic"

    ```bat
    :: activate ROS environment
    c:\opt\ros\melodic\x64\setup.bat

    :: checkout MoveIt tutorial packages
    mkdir c:\moveit_ws\src
    cd c:\moveit_ws\src

    git clone https://github.com/ros-planning/moveit_tutorials.git -b melodic-devel
    git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel
    git clone https://github.com/frankaemika/franka_ros-release -b release/kinetic/franka_description

    :: build packages
    cd c:\moveit_ws
    catkin_make
    ```

After it is built, activate the development workspace.

```bat
c:\moveit_ws\devel\setup.bat
```

## Getting Started with MoveIt Tutorials
Now you are ready to continue to the MoveIt Quickstart in RViz section of the tutorials [MoveIt Tutorials](https://ros-planning.github.io/moveit_tutorials/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html).
