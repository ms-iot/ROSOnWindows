
# Continuous Integration with Github Actions
Actions are a feature of Github which allows you to easily integrate continuous integration into your builds. The Tooling Working Group of the ROS2 technical steering committee has built a [Github action](https://github.com/ros-tooling/action-ros-ci) &nearr;  which allows you to build ROS packages. Microsoft is contributing to this action.

The ROS action has different setups for ROS1 and ROS2, which will be covered below.

## ROS1 Setup
To configure your ROS1 repository for CI, you'll need to install the buildtime files which configures your solution for ROS building. 

Initialize Dependencies and Toolchain

  * On github, select the `Create new file` button.
  * Name the file `ci/deps.rosinstall`
  * In that file, place the contents of [deps.rosinstall](deps.md) into the newly created file
  * Name the file `ci/empty.rosinstall`
  * In that file, place the contents of [empty.rosinstall](empty.md) into the newly created file
  * On github, select the `Create new file` button.
  * Name the file `ci/defaults.yaml`
  * In that file, place the contents of [defaults.yaml](defaults.md) into the newly created file
  * Name the file `ci/packaging.yaml`
  * In that file, place the contents of [packaging.yaml](packaging.md) into the newly created file
  * On github, select the `Create new file` button.
  * Name the file `ci/environment.yaml`
  * In that file, place the contents of [environment.yaml](environment.md) into the newly created file
  * On github, select the `Create new file` button.
  * Name the file `ci/toolchain.cmake`
  * In that file, place the contents of [toolchain.cmake](toolchain.md) into the newly created file


Now create the Github Action

  * On github, select the `Action` tab.
  * Create a new workflow
  * In the new workflow, copy the contents of [main.yaml](ros1_workflow.md) to the newly created workflow file.
  * Replace `<ros package>` with the ROS package you are generating

## ROS2 Setup
To configure your ROS2 repository for CI, you'll need to install the buildtime files which configures your solution for ROS building. 

Now create the Github Action

  * On github, select the `Action` tab.
  * Create a new workflow
  * In the new workflow, copy the contents of [main.yaml](ros2_workflow.md) to the newly created workflow file.
  * Replace `<ros package>` with the ROS package you are generating