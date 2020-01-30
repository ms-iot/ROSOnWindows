# Using a ROS Package on Windows
The ROS community has many thousands of [packages][package stats] which cover many different aspects of building a Robot. Many nodes have been ported by the community or build on Windows without modification. 

## Is the package already supported?
To consume a ROS package, we recommend the following workflow:

### Binary Installation

Determine if there is a binary release of the ROS node.

* Using [ROS Wiki](http://wiki.ros.org) &nearr;, locate the binary release name and attempt to install using `Chocolatey`.
* If this succeeds, then you are all set!
```no-highlight
choco install ros-melodic-<project>
```

### Source Installation

If there isn't a binary release, determine if there is a source only distribution.

* Look on [`https://wiki.ros.org/<project>`][wiki link] or [`https://index.ros.org/`][index link] for the package you are interested in.
* Navigate to the project repository.
* Look for `windows`, `init_windows`, or current development branch to use.
* Clone that branch locally into a catkin workspace:
```no-highlight
mkdir c:\catkin_ws\src
cd c:\catkin_ws\src
git clone -b init_windows http://github.com/<organization>/<project>
```

If Windows is not enabled for the ROS package, check to see if Microsoft's ms-iot Github organization has a fork of that project and working on a port.

* See if there is a `<project>` fork on [http://github.com/ms-iot](http://github.com/ms-iot)
* If it does, clone that into your catkin workspace:
```no-highlight
mkdir c:\catkin_ws\src
cd c:\catkin_ws\src
git clone -b init_windows http://github.com/ms-iot/<project>
```

### Contribute

* If the node has not been enabled on Windows, please create an issue on the ROS package's project page asking for Windows to be supported. 

* If you are able, please consider enabling the ROS package on Windows and submitting a pull request to the original repository. Please refer to the [ROS porting guide](../Porting/Cookbook.md)

[package stats]: https://index.ros.org/stats/
[wiki link]: https://wiki.ros.org/
[index link]: https://index.ros.org/
