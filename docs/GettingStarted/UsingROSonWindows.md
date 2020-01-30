---
title: Using a ROS Package on Windows
---

The ROS community has many thousands of [packages][package stats] which cover many different aspects of building a Robot.
Many packages have been ported by the community or build on Windows without modification.

## Is the package already available?

To consume a ROS package, we recommend the following workflow:

## Binary Installation

Determine if there is a binary release of the ROS node.

* Using [ROS Wiki](http://wiki.ros.org) &nearr;, locate the binary release name and attempt to install using `Chocolatey`.
* If this succeeds, then you are all set!
```no-highlight
choco install ros-melodic-<pacakge_name>
```

## Source Installation

If there isn't a binary release, determine if there is a source only distribution.
Here is an example workflow how to create a workspace to test the availability:

```bash
:: activate the ROS environment
c:\opt\ros\melodic\x64\setup.bat

:: create a empty workspace
mkdir c:\catkin_ws\src
cd c:\catkin_ws

:: generate the released package sources list and its ROS dependencies
:: you can customize the command line to checkout the sources from different channels
:: see the tips section for more details
rosinstall_generator <package_name> --deps --exclude RPP --tar --flat > pkg.rosinstall

:: you can manually edit the pkg.rosinstall for more customizations.
:: see the tips section for more details

:: checkout the sources for real
wstool init src
wstool merge -r -y -t src pkg.rosinstall
wstool update -t src

:: attempt to acquire the external dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

:: now catkin make to build the workspace
catkin_make
```

If everything goes well, now you can activate the development space and use the package.

```
:: activate the development space
devel\setup.bat

:: test the new package is discoverable
rospack find <package_name>
```

## Tips for Source Installation

If something is not successfully built, you can try to the following steps:

* In such case where the Windows port is not yet released for a package, you can repeat [Soruce Installation](#source-installation) but ask `rosinstall_generator` to use the development branch this time.
```bash
:: use the development branch
rosinstall_generator <package> --upstream-development --deps --exclude RPP > pkg.rosinstall
```

* If using the upstream development branch doesn't help, you can edit `pkg.rosinstall` to switch the `version:` of the broken package to other branch, like `windows` or `init_windows`, if it exists.

* In additions to consuming the upstream repositories, you can check to see if [Microsoft's ms-iot Github organization][ms-iot ros repos] has a fork of that project and is working on a port.
  If it does, you can edit `pkg.rosinstall` to point `uri:` to the different fork.

* For the package not registered to the [ROS distributions][rosdistro], you can manually create a [`.rosinstall`][rosinstall] file to maintain a list of repositories to consume.

* [ROS Wiki][wiki link] or [ROS Index][index link] are also good resources to search for the package repository.

## Contribute

If the package has not been enabled on Windows, please create an issue on the ROS package's project page asking for Windows to be supported. 

If you are able, please consider enabling the ROS package on Windows and submitting a pull request to the original repository. [Porting a ROS Package](PortingANode.md) is a good resource to learn the how-to.

[package stats]: https://index.ros.org/stats/
[wiki link]: https://wiki.ros.org/
[index link]: https://index.ros.org/
[ms-iot ros repos]: https://github.com/search?p=7&q=topic%3Aros+fork%3Atrue+org%3Ams-iot&type=Repositories
[rosdistro]: https://github.com/ros/rosdistro
[rosinstall]: https://www.ros.org/reps/rep-0126.html
