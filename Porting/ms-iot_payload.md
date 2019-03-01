# ms-iot payloads for ROS on Windows packages

## List of repos redirected in [rosdistro-db (ms-iot)](https://github.com/ms-iot/rosdistro-db/blob/init_windows/melodic/distribution.yaml):

**TODO: remove redirection to ms-iot repos without change**

*all repos in `init_windows` branch unless specified*
* https://github.com/ms-iot/abseil-cpp.git
* https://github.com/ms-iot/actionlib.git
* https://github.com/ms-iot/angles.git
* https://github.com/ms-iot/bond_core.git
* https://github.com/ms-iot/catkin.git
* https://github.com/ms-iot/class_loader.git
* https://github.com/ms-iot/common_msgs.git
* https://github.com/ms-iot/control_toolbox.git
* https://github.com/ms-iot/diagnostics.git
* https://github.com/ms-iot/dynamic_reconfigure.git
* https://github.com/ms-iot/filters.git
* https://github.com/ms-iot/gazebo_ros_pkgs.git
* https://github.com/ms-iot/gencpp.git
* https://github.com/ms-iot/geometric_shapes.git
* https://github.com/ms-iot/geometry.git
* https://github.com/ms-iot/geometry2.git
* https://github.com/ms-iot/image_common.git
* https://github.com/ms-iot/image_pipeline.git
* https://github.com/ms-iot/industrial_core.git
* https://github.com/ms-iot/interactive_markers.git
* https://github.com/ms-iot/joint_state_publisher.git
* https://github.com/ms-iot/joystick_drivers.git
* https://github.com/ms-iot/kdl_parser.git
* https://github.com/ms-iot/laser_geometry.git
* https://github.com/ms-iot/metapackages.git
* https://github.com/ms-iot/moveit.git
* https://github.com/ms-iot/moveit_visual_tools.git
* https://github.com/ms-iot/navigation.git
* https://github.com/ms-iot/nodelet_core.git
* https://github.com/ms-iot/open_karto.git
* https://github.com/ms-iot/orocos_kinematics_dynamics.git
* https://github.com/ms-iot/perception_pcl.git
* https://github.com/ms-iot/pluginlib.git
* https://github.com/ms-iot/python_qt_binding.git
* https://github.com/ms-iot/qt_gui_core.git
* https://github.com/ms-iot/random_numbers.git
* https://github.com/ms-iot/realtime_tools.git
* https://github.com/ms-iot/resource_retriever.git
* https://github.com/ms-iot/robot_state_publisher.git
* https://github.com/ms-iot/ros.git
* https://github.com/ms-iot/ros_comm.git
* https://github.com/ms-iot/ros_control.git
* https://github.com/ms-iot/ros_controllers.git
* https://github.com/ms-iot/ros_environment.git
* https://github.com/ms-iot/ros_tutorials.git
* https://github.com/ms-iot/ros_type_introspection.git
* https://github.com/ms-iot/rosconsole.git
* https://github.com/ms-iot/rosconsole_bridge.git
* https://github.com/ms-iot/roscpp_core.git
* https://github.com/ms-iot/rospack.git
* https://github.com/ms-iot/rosserial.git
* https://github.com/ms-iot/rqt.git
* https://github.com/ms-iot/rqt_bag.git
* https://github.com/ms-iot/rqt_console.git
* https://github.com/ms-iot/rqt_dep.git
* https://github.com/ms-iot/rqt_graph.git
* https://github.com/ms-iot/rqt_image_view.git
* https://github.com/ms-iot/rqt_logger_level.git
* https://github.com/ms-iot/rqt_plot.git
* https://github.com/ms-iot/rqt_rviz.git
* https://github.com/ms-iot/rqt_shell.git
* https://github.com/ms-iot/rviz.git
* https://github.com/ms-iot/rviz_visual_tools.git
* https://github.com/ms-iot/slam_karto.git
* https://github.com/ms-iot/sparse_bundle_adjustment.git
* https://github.com/ms-iot/srdfdom.git
* https://github.com/ms-iot/stage-release.git
* https://github.com/ms-iot/stage_ros.git
* https://github.com/ms-iot/teleop_twist_joy.git
* https://github.com/ms-iot/universal_robot.git
* https://github.com/ms-iot/urdf.git
* https://github.com/ms-iot/vision_opencv.git
* https://github.com/ms-iot/visualization_tutorials.git
* https://github.com/ms-iot/warehouse_ros.git
* https://github.com/ms-iot/xacro.git
* https://github.com/ms-iot/descartes

## list of `init_windows` repos for current porting task with cached commits:

### 1st Tier Changes

Packages in `ros-melodic-ros_base`.

`ros`
* [~~ms-iot/rosdistro-db~~](https://github.com/ros/rosdistro/compare/master...ms-iot:init_windows)

`ros-infrastructure`
* [ms-iot/catkin_pkg](https://github.com/ros-infrastructure/catkin_pkg/compare/master...ms-iot:init_windows)
* [ms-iot/rosdep](https://github.com/ros-infrastructure/rosdep/compare/master...ms-iot:init_windows)
* [ms-iot/rosinstall_generator](https://github.com/ros-infrastructure/rosinstall_generator/compare/master...ms-iot:init_windows)
* [ms-iot/wstool](https://github.com/vcstools/wstool/compare/master...ms-iot:init_windows)
* [ms-iot/vcstools](https://github.com/vcstools/vcstools/compare/master...ms-iot:init_windows)

`catkin`
* [ms-iot/catkin](https://github.com/ros/catkin/compare/kinetic-devel...ms-iot:init_windows)

`ros_core`
* [ms-iot/class_loader](https://github.com/ros/class_loader/compare/melodic-devel...ms-iot:init_windows)
* [~~ms-iot/common_msgs~~](https://github.com/ros/common_msgs/compare/jade-devel...ms-iot:init_windows)
* [~~ms-iot/gencpp~~](https://github.com/ros/gencpp/compare/indigo-devel...ms-iot:init_windows)
* [ms-iot/pluginlib](https://github.com/ros/pluginlib/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/ros](https://github.com/ros/ros/compare/kinetic-devel...ms-iot:init_windows)
* [ms-iot/ros_comm](https://github.com/ros/ros_comm/compare/melodic-devel...ms-iot:init_windows)
* [~~ms-iot/rosconsole~~](https://github.com/ros/rosconsole/compare/melodic-devel...ms-iot:init_windows)
* [~~ms-iot/rosconsole_bridge~~](https://github.com/ros/rosconsole_bridge/compare/kinetic-devel...ms-iot:init_windows)
* [~~ms-iot/roscpp_core~~](https://github.com/ros/roscpp_core/compare/kinetic-devel...ms-iot:init_windows)
* [~~ms-iot/rospack~~](https://github.com/ros/rospack/compare/lunar-devel...ms-iot:init_windows)
    * dependency
    * [ms-iot/ros_environment](https://github.com/ros/ros_environment/compare/melodic...ms-iot:init_windows)

`ros_base`
* [~~ms-iot/actionlib~~](https://github.com/ros/actionlib/compare/indigo-devel...ms-iot:init_windows)
* [ms-iot/bond_core](https://github.com/ros/bond_core/compare/kinetic-devel...ms-iot:init_windows)
* [ms-iot/dynamic_reconfigure](https://github.com/ros/dynamic_reconfigure/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/nodelet_core](https://github.com/ros/nodelet_core/compare/indigo-devel...ms-iot:init_windows)

### 2nd Tier Changes

Packages beyond `ros-melodic-ros_base`, within `ros-melodic-desktop_full`.

`ros`
* [ms-iot/diagnostics](https://github.com/ros/diagnostics/compare/indigo-devel...ms-iot:init_windows), need to merge https://github.com/ros/diagnostics/pull/96 after
* [~~ms-iot/filters~~](https://github.com/ros/filters/compare/lunar-devel...ms-iot:init_windows)
* [ms-iot/geometry](https://github.com/ros/geometry/compare/melodic-devel...ms-iot:init_windows)
* [~~ms-iot/joint_state_publisher~~](https://github.com/ros/joint_state_publisher/compare/kinetic-devel...ms-iot:init_windows)
* [ms-iot/kdl_parser](https://github.com/ros/kdl_parser/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/robot_state_publisher](https://github.com/ros/robot_state_publisher/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/urdf](https://github.com/ros/urdf/compare/melodic-devel...ms-iot:init_windows)
* [~~ms-iot/xacro~~](https://github.com/ros/xacro/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/geometry2](https://github.com/ros/geometry2/compare/melodic-devel...ms-iot:init_windows)
* [~~ms-iot/resource_retriever~~](https://github.com/ros/resource_retriever/compare/kinetic-devel...ms-iot:init_windows)
* [~~ms-iot/angles~~](https://github.com/ros/angles/compare/master...ms-iot:init_windows)
* [ms-iot/ros_tutorials](https://github.com/ros/ros_tutorials/compare/melodic-devel...ms-iot:init_windows)

`ros-perception`
* [ms-iot/image_common](https://github.com/ros-perception/image_common/compare/hydro-devel...ms-iot:init_windows)
* [~~ms-iot/image_pipeline~~](https://github.com/ros-perception/image_pipeline/compare/indigo...ms-iot:init_windows)
* [ms-iot/perception_pcl](https://github.com/ros-perception/perception_pcl/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/vision_opencv](https://github.com/ros-perception/vision_opencv/compare/melodic...ms-iot:init_windows)
* [~~ms-iot/laser_geometry~~](https://github.com/ros-perception/laser_geometry/compare/indigo-devel...ms-iot:init_windows)

`ros-visualization`
* [ms-iot/rqt](https://github.com/ros-visualization/rqt/compare/kinetic-devel...ms-iot:init_windows)
* [ms-iot/qt_gui_core](https://github.com/ros-visualization/qt_gui_core/compare/kinetic-devel...ms-iot:init_windows)
* [~~ms-iot/rqt_bag~~](https://github.com/ros-visualization/rqt_bag/compare/master...ms-iot:init_windows)
* [~~ms-iot/rqt_console~~](https://github.com/ros-visualization/rqt_console/compare/master...ms-iot:init_windows)
* [~~ms-iot/rqt_dep~~](https://github.com/ros-visualization/rqt_dep/compare/master...ms-iot:init_windows)
* [~~ms-iot/rqt_graph~~](https://github.com/ros-visualization/rqt_graph/compare/master...ms-iot:init_windows)
* [~~ms-iot/rqt_image_view~~](https://github.com/ros-visualization/rqt_image_view/compare/master...ms-iot:init_windows)
* [~~ms-iot/rqt_logger_level~~](https://github.com/ros-visualization/rqt_logger_level/compare/master...ms-iot:init_windows)
* [~~ms-iot/rqt_plot~~](https://github.com/ros-visualization/rqt_plot/compare/master...ms-iot:init_windows)
* [~~ms-iot/rqt_shell~~](https://github.com/ros-visualization/rqt_shell/compare/master...ms-iot:init_windows)
* [ms-iot/python_qt_binding](https://github.com/ros-visualization/python_qt_binding/compare/kinetic-devel...ms-iot:init_windows)
* [ms-iot/rqt_rviz](https://github.com/ros-visualization/rqt_rviz/compare/lunar-devel...ms-iot:init_windows)
* [ms-iot/rviz](https://github.com/ros-visualization/rviz/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/interactive_markers](https://github.com/ros-visualization/interactive_markers/compare/indigo-devel...ms-iot:init_windows)
* [ms-iot/visualization_tutorials](https://github.com/ros-visualization/visualization_tutorials/compare/kinetic-devel...ms-iot:init_windows)

`ros-simulation`
* [ms-iot/gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs/compare/melodic-devel...ms-iot:init_windows)
* [~~ms-iot/stage_ros~~](https://github.com/ros-simulation/stage_ros/compare/lunar-devel...ms-iot:init_windows)
    * dependency
    * [~~ms-iot/stage-release~~](https://github.com/ros-gbp/stage-release/compare/release/melodic/stage...ms-iot:init_windows)

`ros-controls`
* [~~ms-iot/control_toolbox~~](https://github.com/ros-controls/control_toolbox/compare/melodic-devel...ms-iot:init_windows)
* [~~ms-iot/ros_control~~](https://github.com/ros-controls/ros_control/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/ros_controllers](https://github.com/ros-controls/ros_controllers/compare/melodic-devel...ms-iot:init_windows), only test code change
* [~~ms-iot/realtime_tools~~](https://github.com/ros-controls/realtime_tools/compare/melodic-devel...ms-iot:init_windows)

`ros-planning`
* [ms-iot/navigation](https://github.com/ros-planning/navigation/compare/melodic-devel...ms-iot:init_windows)

`orocos`
* [ms-iot/orocos_kinematics_dynamics](https://github.com/orocos/orocos_kinematics_dynamics/compare/master...ms-iot:init_windows)

### 3rd Tier Changes

Packages beyond `ros-melodic-destkop_full`.

`moveit`
* [ms-iot/moveit](https://github.com/ros-planning/moveit/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/moveit_visual_tools](https://github.com/ros-planning/moveit_visual_tools/compare/melodic-devel...ms-iot:init_windows)
    * dependency
    * [ms-iot/geometric_shapes](https://github.com/ros-planning/geometric_shapes/compare/melodic-devel...ms-iot:init_windows)
    * [ms-iot/random_numbers](https://github.com/ros-planning/random_numbers/compare/master...ms-iot:init_windows)
    * [ms-iot/rviz_visual_tools](https://github.com/PickNikRobotics/rviz_visual_tools/compare/melodic-devel...ms-iot:init_windows)
    * [ms-iot/srdfdom](https://github.com/ros-planning/srdfdom/compare/melodic-devel...ms-iot:init_windows)
    * [ms-iot/warehouse_ros](https://github.com/ros-planning/warehouse_ros/compare/kinetic-devel...ms-iot:init_windows)

`universal_robot`
* [ms-iot/universal_robot](https://github.com/ros-industrial/universal_robot/compare/kinetic-devel...ms-iot:init_windows)

`ros_type_introspection`
* [ms-iot/ros_type_introspection](https://github.com/facontidavide/ros_type_introspection/compare/master...ms-iot:init_windows)
    * dependency
    * [ms-iot/abseil-cpp](https://github.com/Eurecat/abseil-cpp/compare/master...ms-iot:init_windows)

`others`
* [ms-iot/industrial_core](https://github.com/ros-industrial/industrial_core/compare/kinetic-devel...ms-iot:init_windows)
* [ms-iot/joystick_drivers](https://github.com/ros-drivers/joystick_drivers/compare/master...ms-iot:init_windows)
* [~~ms-iot/metapackages~~](https://github.com/ros/metapackages/compare/melodic-devel...ms-iot:init_windows)
* [~~ms-iot/open_karto~~](https://github.com/ros-perception/open_karto/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/rosserial](https://github.com/ros-drivers/rosserial/compare/melodic-devel...ms-iot:init_windows)
* [~~ms-iot/slam_karto~~](https://github.com/ros-perception/slam_karto/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/sparse_bundle_adjustment](https://github.com/ros-perception/sparse_bundle_adjustment/compare/melodic-devel...ms-iot:init_windows)
* [ms-iot/teleop_twist_joy](https://github.com/ros-teleop/teleop_twist_joy/compare/indigo-devel...ms-iot:init_windows)
* [ms-iot/descartes](https://github.com/ros-industrial-consortium/descartes/compare/kinetic-devel...ms-iot:init_windows)

## Other informations
The progress of upstream is tracked under [projects](https://github.com/ms-iot/ROSOnWindows/projects).
### Open pull requests from MSFT contributors
* [pull requests from @johnsonshih](https://github.com/pulls?utf8=%E2%9C%93&q=is%3Aopen+is%3Apr+author%3Ajohnsonshih+archived%3Afalse+)
* [pull requests from @kejxu](https://github.com/pulls?utf8=%E2%9C%93&q=is%3Aopen+is%3Apr+author%3Akejxu+archived%3Afalse+)
* [pull requests from @seanyen-msft](https://github.com/pulls?utf8=%E2%9C%93&q=is%3Aopen+is%3Apr+author%3Aseanyen-msft+archived%3Afalse+)
