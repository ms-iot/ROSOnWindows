# Using a ROS Package on Windows
The ROS community has many thousands of nodes which cover many different aspects of building a Robot. Many nodes have been ported by the community or build on Windows without modification. 

## Is the node already supported?
To consume a ROS node, we recommend the following workflow:

* Determine if there is a binary release of the ROS node.
    * Using [ROS Wiki](http://wiki.ros.org) &nearr;, locate the binary release name and attempt to install using chocolatey. 
    * If this succeeds, then you are all set!
```no-highlight
choco install ros-melodic-<project>
```
* If there isn't a binary release, determine if there is a source only distribution. 
    * Look on `https://wiki.ros.org/<project>` for the node you are interested in.
    * navigate to the project repository.
    * Look to see if the node has a `windows`, `init_windows`, or similar branch

* If it has a `windows`, `init_windows` or similar branch 
   * Clone that branch locally into a catkin workspace
```no-highlight
mkdir c:\ws\test_ws
cd c:\ws\test_ws
git clone -b init_windows http://github.com/<organization>/<project>
```

  * If it does not have a `windows`, `init_windows` or similarly named branch, check to see if the current development branch has already enabled Windows.

  * If Windows is not enabled for the ROS node, check to see if Microsoft's ms-iot Github organization has a fork of that project and working on a port.
      * See if there is a `<project>` fork on http://github.com/ms-iot
      * If it does, clone that into your catkin workspace
```no-highlight
mkdir c:\ws\test_ws
cd c:\ws\test_ws
git clone -b init_windows http://github.com/ms-iot/<project>
```

* If the node has not been enabled on Windows, please create an issue on the ROS node's project page asking for Windows to be supported. 

* If you are able, please consider enabling the ROS node on Windows and submitting a pull request to the original repository. Please refer to the [ROS porting guide](../Porting/Cookbook.md)
