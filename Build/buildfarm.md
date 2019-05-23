# The BuildFarm for ROS on Windows

The BuildFarm is a public service which is made of a set of tools and Azure DevOps pipelines to continuously build\test\deliver Open Robotics [ROS](https://www.ros.org/) for Windows developer community.

## Build Status

| Build Pipeline | Build Status |
|-----|-----|
| ros-catkin-build(melodic) | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/ros-catkin-build%20(melodic)?branchName=master)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=54&branchName=master) |
| ros-colon-build(melodic) | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/ros-colcon-build%20(melodic)?branchName=master)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=59&branchName=master) |
| ros-colon-build(crystal) | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/ros-colcon-build%20(crystal)?branchName=master)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=60&branchName=master) |
| ros-colon-build(dashing) | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/ros-colcon-build%20(dashing)?branchName=master)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=61&branchName=master) |
| rosdep-au-packages | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/rosdep-au-packages/rosdep-au-packages%20CI?branchName=master)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=26&branchName=master) |

## Test Results

| Test Pipeline | Test Status |
|-----|-----|
| runtests.ros.melodic.desktop | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/runtests.ros.melodic.desktop?branchName=master)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=33&branchName=master) |
| runtests.ros.melodic.ros_base | [![Build Status](https://ros-win.visualstudio.com/ros-win/_apis/build/status/runtests.ros.melodic.ros_base?branchName=master)](https://ros-win.visualstudio.com/ros-win/_build/latest?definitionId=8&branchName=master) |

## ROS System Dependencies on Windows

[ROS target platforms](http://www.ros.org/reps/rep-0003.html) defines a set of tools and packages which ROS packages depends on. Those tools and packages are also called ROS system dependencies, and they can be deployed by `rosdep` for any supported platforms.

On Windows, `Chocolatey` is chosen as the default package manager for pre-built packages delivery, `rosdep` is extended to support `Chocolatey` and `pip` on Windows platform, and https://roswin.azurewebsites.net is created to host them for Windows developers.

[`rosdep.yaml`](http://www.ros.org/reps/rep-0111.html) is also extended for Windows. Every **ROS on Windows** environmnet gets additional manifest files. For example, [`win-chocolatey.yaml`](https://github.com/ms-iot/rosdistro-db/blob/init_windows/rosdep/win-chocolatey.yaml) defines what `Chocolatey` or `pip` packages to install when Windows developers uses `rosdep` to resolve dependencies.

### Azure DevOps Pipelines for System Dependencies

Everytime an new package is identified to be onboarded for Windows. The pre-built binaries are generated offline and uploaded to `rosdep-au-packages` repository. It is an automatic packaging repository using [`Chocolatey Automatic Package Updater Module`](https://github.com/majkinetor/au), and the deployment is automated in Azure DevOps.

When a package is added or updated, `rosdep-au-packages CI` pipeline will be triggered, and it starts packaging and generating `.nupkg` files. After the packaging pipeline finishes, `ROSDEP to ROSWIN Public Chocolatey Server` pipeline will be triggered in turn and publishing those newly added\updated packages to https://roswin.azurewebsites.net.

## ROS Build on Windows

The Open Source Robotics Foundation (OSRF) maintains public [buildfarm](http://wiki.ros.org/build.ros.org) for the community. Package maintainers can make use of this public services to release ROS packages in sources or pre-built binaries on certain platforms. Likewise, the Buildfarm for ROS on Windows is an equivalent service to complement Windows developer community.

### Nightly Upstream Build Pipelines

One goal of the buildfarm is to make sure every ROS packages built from the upstream (the latest) source code. It is important to catch any regressions as early as possible.

[`ros-catkin-build/azure-pipelines.yml`](https://dev.azure.com/ros-win/ros-win/_git/ros-windows-build?path=%2Fros-catkin-build%2Fazure-pipelines.yml&version=GBmaster) is the entry point for the build. It kicks off [source installation](../Build/source.md) on Azure DevOps and the binaries are packaged into `Chocolatey` packages.

### Pre-built Binaries Release Pipelines

Whenever a nightly build finishes successfully, a deployment will start in turn, which publishes the `Chocolatey` packages to https://roswin.azurewebsites.net. The nightly builds will be firstly published as `prerelease` packages, and a `prerelease` package can be promoted to a `release` package when it mets quality criteria.

