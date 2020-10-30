# Continuous Integration with Github Actions

Actions are a feature of Github which allows you to easily integrate continuous integration into your builds.

## ROS1 CI Setup

To configure your ROS1 repository for CI, you'll need to install the buildtime files which configures your solution for ROS building. 

Create the github action:

* On github, select the `Actions` tab.
* Create a `New Workflow` and name the workflow `Windows_ROS1.yaml`
* Copy the contents of [Windows_ROS1.yaml](ros1_workflow.md) and replace the contents of the file created above.
* Update the workflow for your ROS component. Use the comments in the file as a guide.

If your ROS node has dependencies on other repositories, vcpkgs or chocolatey packages, add them before catkin_make.

``` batch
        : Additional dependencies
        : For other ROS repos, remove the : and add the clone commands
        : pushd src
        : git clone https://github.com/ms-iot/audio_common
        : popd

        : For other chocolatey packages, remove the : and add the choco packages
        : choco install <package>

        : For vcpkgs, remove the : and add the vcpkg dependencies.
        : vcpkg install <package>
```

### Sample CI files for ROS1

The [Azure Kinect ROS Node](https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/melodic/.github/workflows/main.yml) includes a 3rd Party SDK

## ROS2 Setup

Please visit the documentation for the [action-ros-ci](https://github.com/ros-tooling/action-ros-ci) by the tooling working group.

### Sample CI files for ROS2

The [ONNX ROS Node](https://github.com/ms-iot/ros_msft_onnx/.github/workflows/ci.yml) by Microsoft builds on Linux and Windows for ROS2.
