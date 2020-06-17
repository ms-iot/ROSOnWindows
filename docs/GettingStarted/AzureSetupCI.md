# Continuous Integration with Azure Pipelines

This template helps you to set up a continuous integration (CI) build for your ROS repository with ROS.
Use this template if you are not hosting your code in GitHub, if GitHub Actions are not sufficient or if you need to leverage specific Azure Pipelines features.

## Prerequisites

✔️ Learn the basics of [Getting Started with Azure Pipelines](https://docs.microsoft.com/en-us/azure/devops/pipelines/get-started/?view=azure-devops).

✔️ Learn more about [Azure Pipelines](https://docs.microsoft.com/en-us/azure/devops/pipelines/?view=azure-devops).

## Using the Pipeline

Navigate to your Azure Pipelines project and create a new pipeline.

Select the location of your project. If the project is hosted on GitHub, select the GitHub option and then select the repository. You may need to provide Azure Pipelines access permissions to the repository. Select the correct repository if it isn't already selected and click `Approve and Install`.

When walking through the wizard, select `starter pipeline` and it will create a file of `azure-pipelines.yml` under the root of your ROS repository.

Replace azure-pipelines.yml with the following content:

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
    rosdistro: melodic
    metapackage: desktop
    custom_test_target: 'run_tests'
    platforms:
      - linux
      - windows
```

`resources` defines where to look for this common template. In this example, it defines a Github repository reference to `ms-iot\rosonwindows_ci` and use an `endpoint` to access it. 

Replace `endpoint` to your Github account (or your GitHub service connection name).

`jobs\template` defines what template to be included. In this example, include `build.yml@templates`, which means to refer to the `build.yml` under `ms-iot\rosonwindows_ci` GitHub repository.

Under `template`, there are some parameters to customize your CI build:

* `rosdistro` and `metapackage`: You can use `rosdistro` for what ROS distro and `metapackage` for what the composition to check out for your CI build. In this example, it specifies to use `melodic` ROS distro and check out the ROS packages up to `desktop`.

* `custom_test_target`: For projects which do not have  `run_tests` as default test target, it can be set to a customized test target.

* `platforms`: You can use this to select what platforms to run CI builds. Currently `linux` and `windows` are supported values.

Once the wizard finishes, your ROS package will build using the Azure Pipelines.
