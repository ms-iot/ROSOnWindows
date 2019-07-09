# ms-iot payloads for ROS on Windows packages

## List of repos redirected in [rosdistro-db (ms-iot)](https://github.com/ms-iot/rosdistro-db/blob/init_windows/melodic/distribution.yaml):

**TODO: remove redirection to ms-iot repos without change**

*all repos in `init_windows` branch unless specified*
* https://github.com/ms-iot/abseil-cpp.git
* https://github.com/ms-iot/catkin.git
* https://github.com/ms-iot/diagnostics.git
* https://github.com/ms-iot/filters.git
* https://github.com/ms-iot/gazebo_ros_pkgs.git
* https://github.com/ms-iot/geometry.git
* https://github.com/ms-iot/geometry2.git
* https://github.com/ms-iot/image_common.git
* https://github.com/ms-iot/industrial_core.git
* https://github.com/ms-iot/interactive_markers.git
* https://github.com/ms-iot/joystick_drivers.git
* https://github.com/ms-iot/laser_geometry.git
* https://github.com/ms-iot/moveit.git
* https://github.com/ms-iot/moveit_visual_tools.git
* https://github.com/ms-iot/navigation.git
* https://github.com/ms-iot/pluginlib.git
* https://github.com/ms-iot/ros_comm.git
* https://github.com/ms-iot/ros_environment.git
* https://github.com/ms-iot/ros_tutorials.git
* https://github.com/ms-iot/ros_type_introspection.git
* https://github.com/ms-iot/rosserial.git
* https://github.com/ms-iot/rviz_visual_tools.git
* https://github.com/ms-iot/slam_karto.git
* https://github.com/ms-iot/sparse_bundle_adjustment.git
* https://github.com/ms-iot/stage-release.git
* https://github.com/ms-iot/stage_ros.git
* https://github.com/ms-iot/universal_robot.git
* https://github.com/ms-iot/urdf.git
* https://github.com/ms-iot/vision_opencv.git
* https://github.com/ms-iot/descartes

## list of `init_windows` repos for current porting task with cached commits:

### 1st Tier Changes

Packages in `ros-melodic-ros_base`.

`ros`
* [~~ms-iot/rosdistro-db~~](https://github.com/ros/rosdistro/compare/master...ms-iot:init_windows)

`ros-infrastructure`
* [ms-iot/rosdep](https://github.com/ros-infrastructure/rosdep/compare/master...ms-iot:init_windows)
* [ms-iot/wstool](https://github.com/vcstools/wstool/compare/master...ms-iot:init_windows)
* [ms-iot/vcstools](https://github.com/vcstools/vcstools/compare/master...ms-iot:init_windows)

`catkin`
* [ms-iot/catkin](https://github.com/ros/catkin/compare/kinetic-devel...ms-iot:init_windows)

`ros_core`
* [ms-iot/pluginlib](https://github.com/ros/pluginlib/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/ros](https://github.com/ros/ros/compare/kinetic-devel...ms-iot:init_windows)
* [ms-iot/ros_comm](https://github.com/ros/ros_comm/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/ros_environment](https://github.com/ros/ros_environment/compare/melodic...ms-iot:init_windows)

### 2nd Tier Changes

Packages beyond `ros-melodic-ros_base`, within `ros-melodic-desktop_full`.

`ros`
* [ms-iot/diagnostics](https://github.com/ros/diagnostics/compare/indigo-devel...ms-iot:init_windows), need to merge https://github.com/ros/diagnostics/pull/96 after
* [~~ms-iot/filters~~](https://github.com/ros/filters/compare/lunar-devel...ms-iot:init_windows)
* [ms-iot/geometry](https://github.com/ros/geometry/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/urdf](https://github.com/ros/urdf/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/geometry2](https://github.com/ros/geometry2/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/ros_tutorials](https://github.com/ros/ros_tutorials/compare/melodic-devel...ms-iot:init_windows)

`ros-perception`
* [ms-iot/image_common](https://github.com/ros-perception/image_common/compare/hydro-devel...ms-iot:init_windows)
* [ms-iot/vision_opencv](https://github.com/ros-perception/vision_opencv/compare/melodic...ms-iot:init_windows)
* [~~ms-iot/laser_geometry~~](https://github.com/ros-perception/laser_geometry/compare/indigo-devel...ms-iot:init_windows)

`ros-visualization`
* [ms-iot/interactive_markers](https://github.com/ros-visualization/interactive_markers/compare/indigo-devel...ms-iot:init_windows)

`ros-simulation`
* [ms-iot/gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs/compare/melodic-devel...ms-iot:init_windows)
* [~~ms-iot/stage_ros~~](https://github.com/ros-simulation/stage_ros/compare/lunar-devel...ms-iot:init_windows)
    * dependency
    * [~~ms-iot/stage-release~~](https://github.com/ros-gbp/stage-release/compare/release/melodic/stage...ms-iot:init_windows)

`ros-planning`
* [ms-iot/navigation](https://github.com/ros-planning/navigation/compare/melodic-devel...ms-iot:init_windows)

### 3rd Tier Changes

Packages beyond `ros-melodic-destkop_full`.

`moveit`
* [ms-iot/moveit](https://github.com/ros-planning/moveit/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/moveit_visual_tools](https://github.com/ros-planning/moveit_visual_tools/compare/melodic-devel...ms-iot:init_windows)
    * dependency
    * [ms-iot/rviz_visual_tools](https://github.com/PickNikRobotics/rviz_visual_tools/compare/melodic-devel...ms-iot:init_windows)

`universal_robot`
* [ms-iot/universal_robot](https://github.com/ros-industrial/universal_robot/compare/kinetic-devel...ms-iot:init_windows)

`ros_type_introspection`
* [ms-iot/ros_type_introspection](https://github.com/facontidavide/ros_type_introspection/compare/master...ms-iot:init_windows)
    * dependency
    * [ms-iot/abseil-cpp](https://github.com/Eurecat/abseil-cpp/compare/master...ms-iot:init_windows)

`others`
* [ms-iot/industrial_core](https://github.com/ros-industrial/industrial_core/compare/kinetic-devel...ms-iot:init_windows)
* [ms-iot/joystick_drivers](https://github.com/ros-drivers/joystick_drivers/compare/master...ms-iot:init_windows)
* [ms-iot/rosserial](https://github.com/ros-drivers/rosserial/compare/melodic-devel...ms-iot:init_windows)
* [~~ms-iot/slam_karto~~](https://github.com/ros-perception/slam_karto/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/sparse_bundle_adjustment](https://github.com/ros-perception/sparse_bundle_adjustment/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/descartes](https://github.com/ros-industrial-consortium/descartes/compare/kinetic-devel...ms-iot:init_windows)

## Other informations
The progress of upstream is tracked under [projects](https://github.com/ms-iot/ROSOnWindows/projects).
### Open pull requests from MSFT contributors
* [pull requests from @johnsonshih](https://github.com/pulls?utf8=%E2%9C%93&q=is%3Aopen+is%3Apr+author%3Ajohnsonshih+archived%3Afalse+)
* [pull requests from @kejxu](https://github.com/pulls?utf8=%E2%9C%93&q=is%3Aopen+is%3Apr+author%3Akejxu+archived%3Afalse+)
* [pull requests from @seanyen](https://github.com/pulls?utf8=%E2%9C%93&q=is%3Aopen+is%3Apr+author%3Aseanyen+archived%3Afalse+)
