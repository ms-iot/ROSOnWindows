# Continuous Simulation Lab with GitHub Actions (Windows)

## Overview

After defining your robot behavior, it is important to ensure the robot running as expected as the project iterates.
The traditional unit tests and integration tests usually use the static data to exercise your code.
Continous simulation demonstrates how you make use of `GitHub Actions` and [`ROS on Azure with Windows VM`](https://azure.microsoft.com/en-us/resources/templates/ros-vm-windows/) to run your code in Hardware-in-the-loop simulation and virtual environments.
In this lab, we begin with self-driving car project with Gazebo simulation, test it on the rostest framework, and then cloud-host the continuous simulation on GitHub Actions.

This lab uses the [software](https://github.com/Autonomous-Racing-PG/ar-tu-do) developed by the Autonomous Racing Project Group of [TU Dortmund](https://ls12-www.cs.tu-dortmund.de/daes/).
Credit goes to all the [contributors](https://github.com/Autonomous-Racing-PG/ar-tu-do/graphs/contributors).

## Objectives

* Run and observe the autonomous racecar simulation locally.

* Deploy and register a ROS on Windows GPU optimized virtual machine on Azure.

* Integrate a GitHub project with GitHub Actions.

* Observe the simulation runs and test results on the pipeline.

## Prerequisites

* An Microsoft Azure account from https://portal.azure.com.

* A GitHub account from https://github.com.

## Exercise 1: Build And Run Autonomous Car Simulation Locally

1. Fork [`ms-iot/ros_simulation_lab`](https://github.com/ms-iot/ros_simulation_lab) repository into your GitHub account.
2. Follow this ROS Wiki [page](http://wiki.ros.org/Installation/Windows) to install ROS Melodic on Windows.
3. Open the ROS command prompt, and run the following to build the project.

```bat
:: Clone the github project
git clone https://github.com/<your account>/ros_simulation_lab --recursive
cd ros_simulation_lab

:: install required components
vcpkg install sdl2:x64-windows
pip install circle-fit

:: build it
cd catkin_ws
catkin_make --use-ninja -DCMAKE_BUILD_TYPE=RELEASE
```

4. Run the autonomous car simulation.

```bat
:: source the ROS devel space.
devel\setup.bat

:: run the application
roslaunch src\ar-tu-do\ros_ws\launch\gazebo.launch world:=racetrack mode_override:=2
```

This launch file runs a racecar in a simulated track in Gazebo and runs autonomous driving.

![simulation](https://github.com/ms-iot/ros_simulation_lab/raw/master/docs/simulation.png)

## Exercise 2: Run ROSTest With Autonomous Car Simulation Locally

1. End the previous exercise and run the following rostest file:

```bat
:: source the ROS devel space.
devel\setup.bat

:: run the rostest
rostest demo demo.test
```

This [`demo.test`](https://github.com/ms-iot/ros_simulation_lab/catkin_ws/src/demo/test/demo.test) runs the same task as the previous exercise but in headless mode.
Additonally, it runs a [`demo.py`](https://github.com/ms-iot/ros_simulation_lab/catkin_ws/src/demo/nodes/demo.py) node to kick off a validation on the latest lap time.
The [`demo`](https://github.com/ms-iot/ros_simulation_lab/catkin_ws/src/demo) package demonstrates an example how to organize your robot simulation with the rostest framework.
Now let's move this exercise to cloud-hosted environment with GitHub Actions.

## Exercise 3: Provision Cloud CI Environment With GitHub Actions

### Task 1: Fork this GitHub Project

1. Fork [`ms-iot/ros_simulation_lab`](https://github.com/ms-iot/ros_simulation_lab) repository into your GitHub account.
2. Go to your forked repository and navigate to the `Actions` tab.
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

### Task 4: Observe the GitHub Actions

1. Navigate to the `Actions` tab and make sure workflows are listed there.
   ![github-actions-summary](https://github.com/ms-iot/ros_simulation_lab/raw/master/docs/github-actions-summary.png)

2. Push some changes to the fork.
3. Observe the runs of the workflows.
   And you can explore more on [GitHub Help](https://help.github.com/en/actions/configuring-and-managing-workflows/configuring-a-workflow#about-workflows).

