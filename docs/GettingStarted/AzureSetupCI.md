# Continuous Integration with Azure DevOps
This template helps you to set up a continuous integration (CI) build for your ROS repository with ROS on Windows. Use this template if you are not hosting your code in Github, if Github Actions are not sufficient or if you need to leverage specific Azure DevOps features.

## Prerequisite
  * Make sure you have an Azure DevOps pipeline setup. If you don't, visit [here](https://docs.microsoft.com/en-us/azure/devops/pipelines/get-started/pipelines-sign-up) &nearr; for getting started.
  * Learn how to create Azure DevOps pipeline from [here](https://www.youtube.com/watch?v=NuYDAs3kNV8) &nearr;.
  
## Using the Pipeline
Navigate to your Azure DevOps project and create a new pipeline. 

Select the location of your project. If the project is hosted on GitHub, select the GitHub option and then select the repository. You may need to provide Azure Pipelines access permissions to the repository. Select the correct repository if it isn't already selected and click `Approve and Install`.

When walking through the wizard, select `starter pipeline` and it will create a file of `azure-pipelines.yml` under the root of your ROS repository.

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
