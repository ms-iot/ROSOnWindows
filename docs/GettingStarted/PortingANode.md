# Porting a ROS Package to Windows

A C++ ROS node on Windows is an executable or dll in the case of a nodelet. Often, a ROS node is a wrapper around one or more libraries or applications, which may require a separate Software Development Kit (SDK) or other download. This page will walk through supporting a ROS node on Windows, from porting from Linux to build and publish.

ROS1 Core is supported on Windows starting with the Melodic Morenia release. 
ROS 2 has supported Windows since its first releases.

# Porting a ROS Node
To port a ROS node to Windows, we recommend the following general formula. 

1. Determine if the ROS node supports Windows has a binary or source deployment using the [Using ROS Node](UsingROSonWindows.md) guide.
1. Create a fork, clone locally and create a branch
1. Resolve dependencies
1. `catkin_make` in the workspace
1. Fix Build breaks
1. Fix linker breaks
1. Fix runtime bugs
1. Commit and issue a pull request


# Forking & Cloning.
ROS1 on Windows is enabled starting in the Melodic release of ROS. Windows builds are enabled on all ROS2 distributions. 

In order for a ROS1 node to be ported to Windows, it must first support ROS Melodic. ROS packages typically have a tagged branch for the ROS release if there are release specific changes. Fork the repository into your account and clone the melodic branch into a workspace created for the port. 

1. On github.com, fork `https://github.com/<organization>/<project>` into your personal github repo
1. Create a workspace and clone:
```
mkdir c:\ws\ros_ws\src
cd c:\ws\ros_ws\src
git clone https://github.com/<your github>/<project>
cd ..
```


# Resolve Dependencies
Many ROS packages require dependent libraries. After checking out the sources for a ROS package, the tool [rosdep](http://wiki.ros.org/rosdep) &nearr; is run, which will attempt to resolve package dependencies with binary and source distribution managers.

On Windows, the Binary package manager [Chocolatey](https://chocolatey.org/) &nearr; is used. For Source-code only distributions or dependent libraries, [Micrsoft's VCPkg manager](https://github.com/microsoft/vcpkg) &nearr; manager is used. When you install the desktop_full package, vcpkg is automatically installed and built for you.

In the workspace, use `rosdep` to resolve dependencies

```
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

You may see the following output indicating missing dependencies:
```
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
<package: No definition of [<dependent library>] for OS [windows]
#All required rosdeps installed successfully
```


If there are missing dependencies, follow the steps in [Using VCPKG](UsingVCPKG.md) to resolve them with existing ports or add a port using [Add VCPKG](AddingVCPKG.md) as needed.

Take note of special instructions printed while running `rosdep` - these may be needed in order to link correctly on Windows.

# Missing ROS Messages
Many ROS Messages are not packaged as binary distributions. These can simply be cloned into the repo. 

```
cd c:\ws\ros_ws\src
git clone https://github.com/<dependent package organization>/<depdnent package>
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

# Fixing Build breaks
While building you may encouter build breaks. Often, nodes depend on platform specific header files or features. In order to port these, it is recommended to leverage the cross platform equivelent in Boost, STL and cross platform libaries as part of the migration.

Please refer to [Porting Guide: Platform Differences](/Porting/Cookbook.md) for general help on porting.


