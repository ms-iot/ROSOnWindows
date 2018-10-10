# Porting Status
This page covers the current porting status of ROS1 on Windows.

# Plan
Microsoft has been working with Open Robotics and ROS Industrial Consortium on the ROS for Windows porting logistics. Open Robotics has provided a list (below) of packages which need to be ported in order to not be considered experimental. Once tested and upstreamed, Open Robotics will consider Windows supported.

We are targeting Mid-December to complete the ROS porting effort.



# October 2018
+ rViz
    + Missing panels appear to be a timing issue which also repros on Linux. 
+ MoveIt
    + First Life! We have MoveIt planning paths for a UR-3.
+ Developing a program to close the loop with customers
+ Navigation
    + Working with Juraj Oršulić on Cartographer port
    + Working on other ports
+ Started discussions about upstreaming.


# September 2018
+ Core ROS
    + Core ROS has been ported
+ ROSSerial
    + We are observing some transport failures in the ROSSerial connection.
+ rViz
    + Subpanels, such as Display, do not show up consistently. We believe it is an optimization problem. We're working on this.
+ Gazebo
    + Port has not started Yet. This is a long port, which will start after MoveIt for Windows is completed.
+ Turtlebot3
    + Turtlebot mostly brought up.
    + Navigation in progress
+ Perception
    + Camera nodes
    + Lidar
    + OpenNI porting started.
+ Navigation
    + OpenKarto has been ported, but not tested.
    + gmapping port has not started
    + We will discuss other mapping projects with their maintainers
+ MoveIt
    + MoveIt port started
+ Linux testing
    + ROSComm in progress
+ ROS#
    + Testing has not started, but we believe it will work


# Packages to port

**Top 40**

- [X] python-catkin-pkg
- [ ] python-catkin-pkg-modules
- [X] python-rosdep
- [X] python-rosdistro
- [X] python-rosdistro-modules
- [X] python-rospkg
- [X] python-rospkg-modules
- [X] ros-melodic-actionlib
- [X] ros-melodic-actionlib-msgs
- [ ] ros-melodic-cv-bridge
- [X] ros-melodic-desktop
- [ ] ros-melodic-desktop-full
- [X] ros-melodic-diagnostic-updater
- [ ] ros-melodic-gazebo-plugins
- [ ] ros-melodic-gazebo-ros
- [X] ros-melodic-geometry-msgs
- [X] ros-melodic-image-transport
- [X] ros-melodic-interactive-markers
- [X] ros-melodic-kdl-parser
- [X] ros-melodic-laser-geometry
- [X] ros-melodic-nav-msgs
- [X] ros-melodic-pcl-conversions
- [X] ros-melodic-pcl-msgs
- [X] ros-melodic-pcl-ros
- [X] ros-melodic-robot-state-publisher
- [X] ros-melodic-rqt-robot-plugins
- [X] ros-melodic-rqt-rviz
- [X] ros-melodic-rviz
- [X] ros-melodic-sensor-msgs
- [ ] ros-melodic-simulators
- [X] ros-melodic-tf
- [X] ros-melodic-tf2
- [X] ros-melodic-tf2-eigen
- [X] ros-melodic-tf2-geometry-msgs
- [X] ros-melodic-tf2-kdl
- [X] ros-melodic-tf2-msgs
- [X] ros-melodic-tf2-py
- [X] ros-melodic-tf2-ros
- [X] ros-melodic-tf-conversions
- [X] ros-melodic-viz

