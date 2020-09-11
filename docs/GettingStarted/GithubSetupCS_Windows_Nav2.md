---
title: System Tests with Continuous Simulation on GitHub Actions (Windows)
---

## Overview

After defining your robot behavior, it is important to ensure the robot running as expected as the project iterates.
The traditional unit tests and integration tests usually use the static data to exercise your code.
Continous simulation demonstrates how you make use of `GitHub Actions` and [`ROS on Azure with Windows VM`](https://azure.microsoft.com/en-us/resources/templates/ros-vm-windows/) to run your code in Hardware-in-the-loop simulation and virtual environments.
In this lab, we begin with self-driving car project with Gazebo simulation, test it on the rostest framework, and then cloud-host the continuous simulation on GitHub Actions.

This tutorial is to show how to use `GitHub Actions` to exercise the [Navigation2 System Tests](https://github.com/ros-planning/navigation2/tree/main/nav2_system_tests).
It is a comprehensive End-to-End test pass with Gazebo simulation.
This project is a good example for ROS 2 developers how to organize a End-to-End tests across multiple packages and tools.

## Objectives

* Run and observe the Nav2 system tests running locally.

* Deploy and register a ROS on Windows GPU optimized virtual machine on Azure.

* Integrate a GitHub project with GitHub Actions.

* Observe the simulation runs and test results on the pipeline.

## Prerequisites

* An Microsoft Azure account from https://portal.azure.com.

* A GitHub account from https://github.com.

## Exercise 1: Build And Run Nav2 System Tests Locally

1. [`Install ROS2`](../GettingStarted/SetupRos2.md). `Foxy` is recommended.

2. Open the ROS 2 command prompt and activate the Gazebo environment.

```bat
c:\opt\ros\foxy\x64\setup.bat
c:\opt\ros\foxy\x64\share\gazebo\setup.bat
set SDF_PATH=c:\opt\ros\foxy\x64\share\sdformat\1.6
```

3. Create an empty workspace and clone the Navigation2 repositories. For example,

```bat
:: create an empty workspace
mkdir c:\nav2_ws\src
cd c:\nav2_ws

:: clone the Navigation2
curl https://raw.githubusercontent.com/ms-iot/ROSOnWindows/master/docs/ros2/navigation2_foxy.repos -o navigation2_foxy.repos
vcs import src < navigation2_foxy.repos
```

4. Build the Navigation2 System Tests projects.

```bat
:: then, build the nav2_system_tests
colcon build --packages-select nav2_system_tests
```

A few moment later, a similar message should be put to indicate a successful build:

```
Summary: 1 package finished [2min 10s]
```

> In cases of any build failures, `Log` folder can be found under the workspace. Detailed information can be found there.

5. Run the Nav2 System Tests

```bat
colcon test --packages-select nav2_system_tests
```

6. Verify the test result.

```bat
colcon test-result
```

## Exercise 2: Provision Cloud CI Environment With GitHub Actions

### Task 1: Create a GitHub Repository

1. [Create a repository](https://docs.github.com/en/github/getting-started-with-github/create-a-repo) under your account.
2. Go to your repository and navigate to the `Actions` tab.
3. Make sure it is enabled by your permission.
   ![github-actions](https://github.com/ms-iot/ros_simulation_lab/raw/master/docs/github-actions.png)

### Task 2: Prepare Permission For Self-Hosted GitHub Runner

1. Create a GitHub [personal access token](https://help.github.com/en/github/authenticating-to-github/creating-a-personal-access-token-for-the-command-line) and select the scope of `repo`.
2. Take a note of the personal access token (PAT).

### Task 3: Deploy Virtual Machine and Register as Self-Hosted GitHub Runner

This [`ROS on Azure with Windows VM`](https://azure.microsoft.com/en-us/resources/templates/ros-vm-windows/) is a Azure quickstart template to help setup an Azure virtual machine with ROS installed.

1. Navigate to the template. Click `Deploy to Azure`.
2. A form will be brought to you and here are some important parameters for this exercise.
   * **Virtual Machine Size**: Select `Standard_NV*` for GPU optimized virtual machine. This is required for Gazebo.
   * **Vm Image**: Select `Visual Studio 2019` for the required toolchain to build project.
   * **Pipeline Provider**: Select `GitHubRunner` to register as GitHub Self-hosted Runner.
   * **GitHub Repo**: This is your GitHub account and the repository name seperated by a forward slash. For example, `<your GitHub account>/ros_simulation_lab` is the value of this fork.
   * **GitHub Personal Access Token**: This is the PAT noted from the previous section.

### Task 4: Create the GitHub Workflow Files

The GitHub workflow is a YAML file to define what the steps to take by the GitHub runner.
In this example, you will define one to checkout the code, build it and run tests.

1. Create a file `.github/workflows/build-windows.yml` under your repository.

```yaml
name: Build and Test on Windows
on: [push]

jobs:
  build_and_test:
    runs-on: [self-hosted, windows]
    steps:
    - uses: actions/checkout@v2
      with:
        submodules: recursive
    - name: Prepare
      run: |
        $env:ChocolateyInstall="c:\opt\chocolatey"
        Set-ExecutionPolicy Bypass -Scope Process -Force;
        iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'))
        choco sources add -n=roswin -s https://aka.ms/ros/public --priority 1
        choco install ros-foxy-desktop -y --pre --no-progress -i
      shell: powershell
    - name: Build
      run: |
        call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64
        call "c:\opt\ros\foxy\x64\setup.bat"
        call "c:\opt\ros\foxy\x64\share\gazebo\setup.bat"
        set "SDF_PATH=c:\opt\ros\foxy\x64\share\sdformat\1.6"
        mkdir ws\src
        cd ws
        curl https://raw.githubusercontent.com/ms-iot/ROSOnWindows/master/docs/ros2/navigation2_foxy.repos -o navigation2_foxy.repos
        vcs import src < navigation2_foxy.repos
        colcon build --packages-select nav2_system_tests
        colcon test --packages-select nav2_system_tests --event-handlers console_direct+
        colcon test-result
      shell: cmd
    - name: Archive test results # workaround: https://github.com/actions/upload-artifact/issues/76
      run: zip -r results.zip build
      working-directory: ws
    - name: Upload test results
      uses: actions/upload-artifact@v2
      with:
        path: ws/results.zip
```

2. Commit and push the workflow files to your remote repository.

### Task 4: Observe the GitHub Actions

Now a workflow should be scheduled to run under your repository.

1. Navigate to the `Actions` tab and make sure workflows are listed there.
   ![github-actions-summary](https://github.com/ms-iot/ros_simulation_lab/raw/master/docs/github-actions-summary.png)

2. Observe the runs of the workflows.
   And you can explore more on [GitHub Help](https://help.github.com/en/actions/configuring-and-managing-workflows/configuring-a-workflow#about-workflows).

