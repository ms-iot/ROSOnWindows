---
title: Navigation 2 on Windows
---

[**Navigation 2**][nav2] is the next generation ROS Navigation stack for ROS 2.
Edge robotics team at Microsoft has bootstrapped a Windows port for **Navigation 2**.
This short guide shows you how to build **Navigation 2** from source and later you can get started with **Navigation 2** exercises.

![](./nav2.gif)

## Objectives

  * Exercise the ROS 2 Windows installation.
  * Bootstrap an environment running **Navigation 2** samples.

## Prerequisites

> The instructions can be found on [`http://wiki.ros.org/Installation/Windows`](http://wiki.ros.org/Installation/Windows).

✔️ 64-bit (amd64) environment of `Windows 10 Desktop`.

✔️ `Visual Studio 2019` with `Desktop development with C++` workload included.

✔️ `Chocolatey` package manager installed.

✔️ `Git` source control software installed.

## Installing ROS 2 on Windows

1. From the start menu, look for [`x64 Native Tools Command Prompt for VS 2019`][vsdevcmd].
2. Open the command prompt as administrator.
3. Run the following to install `ROS 2 Foxy`.

```bat
mkdir c:\opt\chocolatey
set ChocolateyInstall=c:\opt\chocolatey
choco source add -n=ros-win -s="https://aka.ms/ros/public" --priority=1
choco upgrade ros-foxy-desktop -y --execution-timeout=0 --pre
```

You can close the command prompt now.

✔️ Now you have ROS 2 `ros-foxy-desktop` installed.

## Open a Developer Command Prompt

1. From the start menu, look for [`x64 Native Tools Command Prompt for VS 2019`][vsdevcmd].
2. Run the shortcut as administrator.
3. Once the developer command prompt is open, run

```bat
:: activate the ROS 2 environment
c:\opt\ros\foxy\x64\setup.bat

:: activate the Gazebo simulation environment
c:\opt\ros\foxy\x64\share\gazebo\setup.bat
set "SDF_PATH=c:\opt\ros\foxy\x64\share\sdformat\1.6"
```

Now you are in the ROS 2 Developer command prompt.
Stay in this command prompt for the rest of this tutorial.

## Create a Navigation 2 Workspace

Create a empty workspace to contain the Navigation 2 project, and then resolve the additional dependencies.

```bat
:: create a empty workspace
mkdir c:\nav2_ws\src
pushd c:\nav2_ws

:: checkout the required source code.
wget https://raw.githubusercontent.com/ms-iot/ROSOnWindows/master/docs/ros2/navigation2_foxy.repos
vcs import src < navigation2.repos
```

## Build and Activate the Navigation 2 Workspace

Build the workspace by `colcon` build tool.

```bat
:: change to the root of workspace
pushd c:\nav2_ws

:: build the workspace
colcon build --cmake-args -DBUILD_TESTING=OFF
```

Activate the workspace which was built.

```bat
:: activate it
install\setup.bat
```

## Verify your environment with Gazebo and TurtleBot3

Now you are in the Navigation 2 activated environment.
Before you explore more, let's run a little exercise to make sure your environment ready to go.

```bat
cd c:\nav2_ws

set GAZEBO_MODEL_PATH=C:\nav2_ws\install\turtlebot3_gazebo\share\turtlebot3_gazebo\models;%GAZEBO_MODEL_PATH%
set TURTLEBOT3_MODEL=waffle

curl -o map.pgm https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/master/turtlebot3_navigation/maps/map.pgm
curl -o map.yaml https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/master/turtlebot3_navigation/maps/map.yaml

start ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
start ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=c:\nav2_ws\map.yaml
```

After a few moment, you should see `TurtleBot3` in a simulation world and the respective map shows in `RViz`.
You can use `2D pose` in `RViz` to give a estimate location to intialize your robot, and use `2D goal` to see Navigation 2 planning a path in action.

## Explore Navigation 2 Samples

There are many **Navigation 2** resources online.
Here we share some good starting points:

* [TurtleBot3 ROS 2 Simulation: Virtual SLAM and Virtual Navigation][turtlebot3ros2]
* [Navigation 2][nav2]


[nav2]: https://ros-planning.github.io/navigation2/
[turtlebot3ros2]: http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_simulation/#ros-2-simulation
[vsdevcmd]: https://docs.microsoft.com/en-us/dotnet/framework/tools/developer-command-prompt-for-vs
