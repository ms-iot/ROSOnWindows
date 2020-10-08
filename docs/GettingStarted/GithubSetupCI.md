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

## ROS2 Setup

To configure your ROS2 repository for CI, you'll need to install the build time files which configures your solution for ROS building. 

Now create the Github Action

* On github, select the `Action` tab.
* Create a new workflow
* In the new workflow, copy the contents of [main.yaml](ros2_workflow.md) to the newly created workflow file.
* Replace `<ros package>` with the ROS package you are generating