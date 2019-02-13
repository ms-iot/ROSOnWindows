# BuildFarm for ROS on Windows

> This page is working in progress.

## Build Status

> Please note, some build failures below are related to in-progress work.

| Variant | Build Status |
|-----|-----|
| ros-melodic-desktop  | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/build.ros.melodic.desktop)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=16)  |
| ros-melodic-desktop_full | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/build.ros.melodic.desktop_full)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=19) |
| ros-melodic-viz | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/build.ros.melodic.viz)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=10) |
| ros-melodic-perception | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/build.ros.melodic.perception)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=21) |
| ros-melodic-robot | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/build.ros.melodic.robot)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=15) |
| ros-melodic-simulators | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/build.ros.melodic.simulators)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=23) |
| ros-melodic-ros_base | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/build.ros.melodic.ros_base)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=5) |
| ros-melodic-ros_core | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/build.ros.melodic.ros_core)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=22) |

| Additional Stack | Build Status |
|-----|-----|
| ros-melodic-moveit | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/build.ros.melodic.moveit?branchName=master)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=29&branchName=master) |
| ros-melodic-cartographer_ros | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/build.ros.melodic.cartographer_ros?branchName=master)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=44&branchName=master) |

## BuildFarm Introduction

There are 3 major components for this BuildFarm:
1. **ROS on Windows** hosted Chocolatey server - https://roswin.azurewebsites.net, where rosdep and **ROS on Windows** pre-built binaries are hosted.
2. Build Pipelines for [Variants for ROS Melodic](http://www.ros.org/reps/rep-0150.html)
3. Build Pipelines for rosdep inventory.

### Build Pipeline for ROS build

To monitor **ROS on Windows** bring-up progress, we use Azure Pipeline as the CI\CD environment.

The main entry for the ROS build process is [BuildROS.vsts-ci.yaml](https://ros-win.visualstudio.com/_git/ros-win?path=%2Ftools%2FBuildROS.vsts-ci.yml&version=GBmaster), and we can break it down into:
1. Setup Cholatey server to https://roswin.azurewebsites.net
2. Walk through the source installation.
3. Pack the binary output into Chocolatey package.

### Build Pipeline for Rosdeps

Additionally, we inventory the rosdep for the system depedencies in a Git repository - [rosdep-au-packages](https://ros-win.visualstudio.com/ros-win/_git/rosdep-au-packages?_a=readme).
