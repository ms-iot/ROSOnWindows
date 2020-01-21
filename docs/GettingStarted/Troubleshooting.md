# Troubleshooting ROS for Windows
The COSINE Robotics feature team is monitoring [ROS Answers](https://answers.ros.org/questions/) &nearr;, [ROS Discourse](https://discourse.ros.org/) &nearr;, and the [/r/ROS Subreddit](http://reddit.com/r/ros) &nearr;.

> This page will be updated with environment related problems as we diagnose them with customers.

# Windows Specific Bugs
Please create a GitHub issue on the [ROS on Windows GitHub repository](https://github.com/ms-iot/ROSOnWindows) &nearr;.

# No Visual Studio command line build?
If you find that you do not have a Visual Studio command line in your start menu, it likely means that it wasn't installed during Visual Studio setup. 

To Fix, please launch the Visual Studio installer and select to install the C++ build environment.

# Failures during Chocolatey install or upgrade
We have seen reports of transient errors during install or upgrade of chocolatey packages or dependencies. If you hit a bug such as :
> ERROR: The running command stopped because the preference variable "ErrorActionPreference" or common parameter is set to Stop: Exception: The upgrade of ros-catkin-tools was NOT successful. Error while running 'C:\ProgramData\chocolatey\lib\ros-catkin-tools\tools\chocolateyinstall.ps1'. See log for details.

Please try running chocolatey upgrade directly on that package:

`choco upgrade ros-catkin-tools`



