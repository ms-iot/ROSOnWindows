# What's New
This page tracks changes to the Azure Edge Robotics Documentation.

## April 2021
### ROS2 Foxy on Hololens
We've been working on migrating a prototype of ROS2 on Hololens code to Foxy. 
This is split into two repositories, [ROS2 Native](http://aka.ms/ros/mrtk_native) and [ROS2 Mixed Reality Toolkit for Unity](http://aka.ms/ros/mrtk). When this work is completed, it will be made available as a nuget package on the Microsoft Chocolatey & Nuget server for ROS, with instructions for installation located on the [Azure Edge Robotics landing Page](https://aka.ms/ros.

### Noetic Installation dependencies
During late April, vcredist2010 was broken on Chocolatey.org, due to a security fix changing the hash of the installer. Because the Microsoft ROS installers depended on this for installation, the chocolatey packages were blocked. We are spinning a new build without this dependency.

### Azure Kinect
We have a large number of Pull Requests in the Azure Kinect which we will address in May.

### Visual Studio Code updates
We [root caused a a memory leak in vscode](https://github.com/ms-iot/vscode-ros/issues/393) and have addressed this with the Visual Studio code team. The next release will fix the core issue in VSCode. Until this is released, if you use VSCode in a container on Ubuntu, please do not enable Port forwarding - instead run ROSCore in the container.

### rosdep for ROS2
The Microsoft ROS2 releases do not automatically include rosdep. We investigated how do appropriately add this to the release. We will include in future releases - starting with a rev of Foxy, and in Galactic.

## March 2021
### ROS Camera node for Windows
We resolved an issue with stride, which impacted some modes of the Azure Kinect.

### ROS2 Rolling and Galactic
Introduced rolling builds and set up for Galactic release of ROS2.
Updated maintainers handbook.

## January 2021
### ROS 2 Launch debugging in VSCode!
Yes, you can now press F5 to launch a ROS2 node from VSCode and step through it.

### Updated Dependencies
OPML with ROS Industrial

### ROS Node bringup for UFactory XArm on ROS on Windows
Brought up in collaboration with UFActory.
https://github.com/ms-iot/xarm_ros


## December 2020
### Refresh ROS1 Melodic, ROS1 Noetic, ROS2 Foxy
Updated across the board.


## November 2020
### Onnx Runtime ROS Node
Cross Platform, hardware accelerated ML ROS node

## October 2020
### ROS nodes in an Azure IoT Edge Container
Demonstrated running ROS nodes in an Azure IoT Edge container

### Azure Kinect ROS Node for ROS2
Azure Kinect now supports ROS2.

## September 2020
### Theme Update!
Update documentation themes.

### More MoveIt on Foxy for Windows
Documentation and build updates for Moveit On Windows

## August 2020
### ROS2 Versions of Microsoft Nodes
Windows ML ROS2 port
Windows Camera node for ROS2

### VM documenation
Updated Virtual Machine Documentation

### MoveIt 2 on Foxy on Windows
https://moveit.ros.org/moveit2/ros2/foxy/release/2020/09/04/moveit2-foxy-release.html

### Continuous Simulation with Scale Set support
Added documentation for using Azure Scale Sets to spin up many simulation instances.

## July 2020
### ROS Distro
Move the Chocolatey distribution to a public redirect
Use Azure DevOps centralized Nuget server.

### ROS2 for Hololens
Bootstrap ROS2 for Hololens

### ROS2.net Fixes
Add ROS2 Eloquent support

### Add Windows Media Foundation ROS Node 
Enables High performance Camera support
Adds support for realtime video streaming for teleoperation
https://github.com/ms-iot/ros_msft_camera


## June 2020
## Simulation
Simulation using Azure DevOps for ROS on Linux
Simulation with Github Agents on Windows
Simulation with Github Agents on Linux
Fixes for Typos

## ROS2
ROS2 binary install documentation update on both http://aka.ms/ros and Open Robotics documentation.

## Azure IoT ROS Node
x.509 Certificate and Device Provisioning Service Support

## Azure Kinect ROS Node
Performance improvements
Multicamera crash fixes

## Commercial Deployment of ROS using MSIx
MSIx is a MDM and intune deployable binary installation.
Added documentation for building a MSIx package for ROS infrastructure and customer install. 

## ROS1 Fixes on Windows
Improved timer repeatability on Windows.

## May 2020
### Simulation with Github
Coming soon

### Continuous Deployment on Azure DevOps

### ROS::Time fixes
We discovered a bug in low level time routines, which affects localization. Corrected and in testing.

### Gmapping
By popular request, we're working on gmapping

## April 2020
### Continuous Deployment on Github
Added documentation for building Windows packages and publishing a chocolatey package as a github release.

### Continuous Simulation Lab
Added a project which describes simulation during build, and tests which validate the build in simulation.

## March 2020
### ROS Github Action
From within Github, you can set up Continuous integration using a Github Action, brought to you by the Tooling Working Group of the ROS2 Technical Steering Committee, with contributions from Microsoft.

### MoveIt Updates
MoveIt Tutorials have been updated to work on Windows.

## February 2020
### Cross platform ROS node updates
We have been adding notes to the porting *cookbook* based on customer feedback. 

### Performance
We've added a section on optimizing performance of Windows for Robotics scenarios. Windows 10 IoT Enterprise LTSC (Long Term Support Channel) is our recommended operating system for Robotics, as it offers the smallest footprint, and includes 10 years of support.

## January 2020
### State of ROS on Windows
ROS1 for Windows was announced generally available in May 2019. Since then it's use has continued to grow. Many companies are porting nodes to Windows. The Visual Studio Code extension has been incredibly well adopted within the community. 

### New Look for Documents
Documentation and Developer experience are a top priority. We're starting off by refreshing the look of docs; following up with turtorials on how to bring solutions up on Windows.


