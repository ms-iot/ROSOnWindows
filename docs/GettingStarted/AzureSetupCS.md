# Continuous Simulation
During the build, unit tests and integration tests are run. These test portions of the codebase usually using static data. With Continuous Simulation, the output of your build is deployed to a **build agent** - a VM or physical machine - which runs your code using hardware-in-the-loop and/or with virtual environments.

Microsoft's Continuous Simulation template leverages existing test infrastructure, bridging between ROS, ROS friendly simulation engines, and DevOps. 

## Coming in Spring 2020
