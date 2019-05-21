# BuildFarm for ROS on Windows

## Build Status

| Build Pipeline | Build Status |
|-----|-----|
| ros-catkin-build(melodic) | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/ros-catkin-build%20(melodic)?branchName=master)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=54&branchName=master) |
| ros-colon-build(all) | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/ros-colcon-build%20(all)?branchName=master)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=55&branchName=master) |
| rosdep-au-packages | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/rosdep-au-packages/rosdep-au-packages%20CI?branchName=master)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=26&branchName=master) |

## Test Results

| Test Pipeline | Test Status |
|-----|-----|
| runtests.ros.melodic.desktop | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/runtests.ros.melodic.desktop?branchName=master)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=33&branchName=master) |
| runtests.ros.melodic.ros_base | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/runtests.ros.melodic.ros_base?branchName=master)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=8&branchName=master) |

## BuildFarm Introduction

There are 3 major components for this BuildFarm:
1. **ROS on Windows** hosted Chocolatey server - https://roswin.azurewebsites.net, where ROS system dependencies (aka rosdep) and **ROS on Windows** pre-built binaries are hosted.
2. Build Pipelines for [Variants for ROS Melodic](http://www.ros.org/reps/rep-0150.html).
3. Build Pipelines for rosdep inventory.

### Build Pipeline for ROS build

To monitor **ROS on Windows** bring-up progress, we use Azure DevOps as the CI\CD environment.

The main entry point for ROS build is [ros-catkin-build/azure-pipelines.yml](https://dev.azure.com/ros-win/ros-win/_git/ros-windows-build?path=%2Fros-catkin-build%2Fazure-pipelines.yml&version=GBmaster), and we can break it down into:
1. Setup Cholatey server to https://roswin.azurewebsites.net
2. Walk through the source installation.
3. Pack the binary output into Chocolatey package.

### Build Pipeline for Rosdeps

Additionally, we inventory the rosdep for the system depedencies in a Git repository - [rosdep-au-packages](https://ros-win.visualstudio.com/ros-win/_git/rosdep-au-packages?_a=readme).
