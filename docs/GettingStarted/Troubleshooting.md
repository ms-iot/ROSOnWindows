# Troubleshooting ROS for Windows
The COSINE Robotics feature team is monitoring [ROS Answers](https://answers.ros.org/questions/) &nearr;, [ROS Discourse](https://discourse.ros.org/) &nearr;, and the [/r/ROS Subreddit](http://reddit.com/r/ros) &nearr;.

> This page will be updated with environment related problems as we diagnose them with customers.

# Windows Specific Bugs
Please create a GitHub issue on the [ROS on Windows GitHub repository](https://github.com/ms-iot/ROSOnWindows) &nearr;.

# No Visual Studio command line build?
If you find that you do not have a Visual Studio command line in your start menu, it likely means that it wasn't installed during Visual Studio setup. 

To Fix, please launch the Visual Studio installer and select to install the C++ build environment.

# CMake Fails to find Visual Studio after upgrade
CMake caches part of the build environment when building a workspace. If you upgrade Visual Studio after building, you may encounter an error like this:

```
CMake Error in CMakeLists.txt:
  The CMAKE_CXX_COMPILER:

    C:/Program Files (x86)/Microsoft Visual Studio/2019/Community/VC/Tools/MSVC/14.22.27905/bin/Hostx64/x64/cl.exe
```

To correct this situation, please delete the devel, build and install directories and rebuild:
```
cd c:\catkin_ws
rd /s build devel install
```

# Failures during Chocolatey install or upgrade
We have seen reports of transient errors during install or upgrade of chocolatey packages or dependencies. If you hit a bug such as :
> ERROR: The running command stopped because the preference variable "ErrorActionPreference" or common parameter is set to Stop: Exception: The upgrade of ros-catkin-tools was NOT successful. Error while running 'C:\ProgramData\chocolatey\lib\ros-catkin-tools\tools\chocolateyinstall.ps1'. See log for details.

Please try running chocolatey upgrade directly on that package:

`choco upgrade ros-catkin-tools`



