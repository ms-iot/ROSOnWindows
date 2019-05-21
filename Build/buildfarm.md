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

## ROS System Dependencies on Windows

The [ROS target platforms](http://www.ros.org/reps/rep-0003.html) defines a set of tools and packages which many ROS core packages depends on. Those packages and tools are also called ROS system dependencies, and they can be deployed by `rosdep` for any supported platforms.

On Windows, `Chocolatey` is chosen as the default package manager for pre-built packages delivery, `rosdep` is extended to support `Chocolatey` and `pip` on Windows platform, and https://roswin.azurewebsites.net is created to host them for Windows developers.

[`rosdep.yaml`](http://www.ros.org/reps/rep-0111.html) is also extended for Windows. Every **ROS on Windows** environmnet gets additional manifest files. For example, [`win-chocolatey.yaml`](https://github.com/ms-iot/rosdistro-db/blob/init_windows/rosdep/win-chocolatey.yaml) defines what `Chocolatey` or `pip` packages to install when Windows developers uses `rosdep` to resolve dependencies.

### Azure DevOps Pipelines for System Dependencies

Everytime an new package is identified to be onboarded for Windows. The pre-built binaries are generated offline and uploaded to `rosdep-au-packages` repository. It is an automatic packaging repository using [`Chocolatey Automatic Package Updater Module`](https://github.com/majkinetor/au), and the deployment is automated in Azure DevOps.

When a package is added or updated, `rosdep-au-packages CI` pipeline will be triggered, and it starts packaging and generating `.nupkg` files. After the packaging pipeline finishes, `ROSDEP to ROSWIN Public Chocolatey Server` pipeline will be triggered in turn and publishing those newly added\updated packages to https://roswin.azurewebsites.net.

## ROS Build on Windows

To monitor **ROS on Windows** bring-up progress, we use Azure DevOps as the CI\CD environment.

The main entry point for ROS build is [ros-catkin-build/azure-pipelines.yml](https://dev.azure.com/ros-win/ros-win/_git/ros-windows-build?path=%2Fros-catkin-build%2Fazure-pipelines.yml&version=GBmaster), and we can break it down into:
1. Setup Cholatey server to https://roswin.azurewebsites.net
2. Walk through the source installation.
3. Pack the binary output into Chocolatey package.
