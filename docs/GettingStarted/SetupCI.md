
# Continuous Integration with Github Actions
Actions are a feature of Github which allows you to easily integrate continuous integration into your builds. The Tooling Working Group of the ROS2 technical steering committee has built a [Github action](https://github.com/ros-tooling/action-ros-ci) &nearr;  which allows you to build ROS packages. Microsoft is contributing to this action.

The ROS action has different setups for ROS1 and ROS2, which will be covered below.

## ROS1 Setup
To configure your ROS1 repository for CI, you'll need to install the buildtime files which configures your solution for ROS building. 

Initialize Dependencies and Toolchain

  * On github, select the `Create new file` button.
  * Name the file `ci/deps.rosinstall`
  * In that file, place the contents of [deps.rosinstall](deps.md) into the newly created file
  * On github, slect the `Create new file` button.
  * Name the file `ci/defaults.yaml`
  * In that file, place the contents of [defaults.yaml](defaults.md) into the newly created file
  * On github, slect the `Create new file` button.
  * Name the file `ci/environment.yaml`
  * In that file, place the contents of [environment.yaml](environment.md) into the newly created file
  * On github, slect the `Create new file` button.
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


# Continuous Integation with Azure DevOps
This template helps you to set up a continuous integration (CI) build for your ROS repository with ROS on Windows. Use this template if you are not hosting your code in Github, if Github Actions are not sufficient or if you need to leverage specific Azure DevOps features.

## Prerequisite
  * Make sure you have an Azure DevOps pipeline setup. If you don't, visit [here](https://docs.microsoft.com/en-us/azure/devops/pipelines/get-started/pipelines-sign-up) &nearr; for getting started.
  * Learn how to create Azure DevOps pipeline from [here](https://www.youtube.com/watch?v=NuYDAs3kNV8) &nearr;.
  
## Using the Pipeline
Navigate to your Azure DevOps project and create a new pipeline. When walking through the wizard, select `starter pipeline` and it will create a file of `azure-pipelines.yml` under the root of your ROS repository. 

Replace `azure-pipelines.yml` with the following content:
```yaml
resources:
  repositories:
    - repository: templates
      type: github
      name: ms-iot/rosonwindows_ci
      endpoint: <your github account>

jobs:
- template: build.yml@templates  # Template reference
  parameters:
    ros_metapackage: 'ros-melodic-desktop'
```

`resources` defines where to look for this common template. In this example, it defines a Github repository reference to `ms-iot\rosonwindows_ci` and use an `endpoint` to access it. 

Replace `endpoint` to your Github account (or your GitHub service connection name).

`jobs\template` defines what template to be included. In this example, include `build.yml@templates`, which means to refer to the `build.yml` under `ms-iot\rosonwindows_ci` GitHub repository.

Under `template`, there are some parameters to customize your CI build:
* `ros_metapackage`: It is the basic image to check out for CI build. In this example, it will install `ros-melodic-desktop` before the CI build.
* `custom_test_target`: For projects which do not have  `run_tests` as default test target, it can be set to a customized test target.

Once the wizard finishes, your ROS package will build using the azure pipeline.
