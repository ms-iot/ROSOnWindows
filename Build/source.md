# Installing from source (ROS for Windows)
Install from source requires that you download and compile the source code on your own.

> This page is organized as the same way as [ROS\Installation\Source](http://wiki.ros.org/Installation/Source). This page is still working in progress.

## Prerequisites
### Installing toolchains
Follow the Prerequisites section from [README](/README.md) to install Visual Studio Build Tool 2017, Cmake, and Chocolatey.

> Please note that *ROS for Windows* expects the 64-bit Python2.7 to be installed under c:\python27amd64.

### Installing bootstrap dependencies
These tools are used to facilitate the download and management of ROS packages and their dependencies, among other things.

**Generic (pip):**

If you are using a non-Debian system you need to make sure that you have all build tools (compiler, CMake, etc.) installed. You can install all ROS Python tools via PIP:

    pip install -U rosdep rosinstall_generator wstool rosinstall
    curl --output requirements.txt -L https://raw.githubusercontent.com/ms-iot/rosdistro-db/init_windows/rosdistro_cache/catkin-requirements.txt
    pip install -U --no-deps --force-reinstall -r requirements.txt

If there are errors with this or the rosdep step below, your system's version of pip may be out-of-date. Use your system's package management to update it, or use it to update itself:
    
    python -m pip install -U pip setuptools

### Initializing rosdep
    rosdep init
    curl --output 10-ms-iot.list -L https://raw.githubusercontent.com/ms-iot/rosdistro-db/init_windows/rosdep/sources.list.d/10-ms-iot.list
    copy 10-ms-iot.list c:\etc\ros\rosdep\sources.list.d
    rosdep update

The `10-ms-iot.list` points to the rosdep database of ROS for Windows and it will be evaulated before the default source list.

### Configure Chocolatey sources
    choco source add -n=ros-win -s="https://roswin.azurewebsites.net/api/v2" --priority=1
    choco source disable -n=chocolatey

This will add roswin Chocolatey server as a source to discover libraries and tools. Also disable the default one to avoid any potential package naming conflicts.

## Installation
Start by building the core ROS packages.

### Create a catkin Workspace
In order to build the core packages, you will need a catkin workspace. Create one now:

    mkdir c:\ros_catkin_ws
    cd c:\ros_catkin_ws

Next we will want to fetch the core packages so we can build them. We will use wstool for this. Select the wstool command for the particular variant you want to install:

**Desktop Install (recommended):** ROS, rqt, rviz, and robot-generic libraries

    set ROSDISTRO_INDEX_URL=https://raw.githubusercontent.com/ms-iot/rosdistro-db/init_windows/index.yaml
    rosinstall_generator desktop --rosdistro melodic --deps --upstream-development > melodic-desktop.rosinstall
    wstool init src melodic-desktop.rosinstall

**ROS-Comm: (Bare Bones)** ROS package, build, and communication libraries. No GUI tools.

    set ROSDISTRO_INDEX_URL=https://raw.githubusercontent.com/ms-iot/rosdistro-db/init_windows/index.yaml
    rosinstall_generator ros_comm --rosdistro melodic --deps --upstream-development > melodic-ros_comm.rosinstall
    wstool init src melodic-ros_comm.rosinstall

This will add all of the catkin packages in the given variant and then fetch the sources into the ~/ros_catkin_ws/src directory. The command will take a few minutes to download all of the core ROS packages into the src folder.

### Resolving Dependencies
Before you can build your catkin workspace you need to make sure that you have all the required dependencies. We use the rosdep tool for this:

    rosdep install --from-paths src --ignore-src --rosdistro melodic -r -y

This will look at all of the packages in the src directory and find all of the dependencies they have. Then it will recursively install the dependencies.

The --from-paths option indicates we want to install the dependencies for an entire directory of packages, in this case src. The --ignore-src option indicates to rosdep that it shouldn't try to install any ROS packages in the src folder from the package manager, we don't need it to since we are building them ourselves. The --rosdistro option is required because we don't have a ROS environment setup yet, so we have to indicate to rosdep what version of ROS we are building for. Finally, the -y option indicates to rosdep that we don't want to be bothered by too many prompts from the package manager.

After a while (and maybe some prompts for your password) rosdep will finish installing system dependencies and you can continue.

### Building the catkin Workspace
Once it has completed downloading the packages and resolving the dependencies you are ready to build the catkin packages. We will use the catkin_make_isolated command because there are both catkin and plain cmake packages in the base install, when developing on your catkin only workspaces you may choose to use [catkin/commands/catkin_make](http://wiki.ros.org/catkin/commands/catkin_make) which only works with catkin packages.

Invoke catkin_make_isolated:

    set PATH=c:\opt\rosdeps\x64\bin;%PATH%
    python .\src\catkin\bin\catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --use-nmake

>The rosdep BIN path needs to be added to `PATH` so catkin can find the build tools (e.g. CMake).

Now the packages should have been installed to c:\ros_catkin_ws\install_isolated or to wherever you specified with the --install-space argument. If you look in that directory you will see that a setup.bash file have been generated. To utilize the things installed there simply source that file like so:
 
    c:\ros_catkin_ws\install_isolated\setup.bat


