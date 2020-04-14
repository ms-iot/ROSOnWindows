```batch
@echo off
setlocal

:: source ROS environments
pushd %~dp0
call "setup.bat"

:: use Python from ROS installation
set PATH=%PYTHONHOME%;%PYTHONHOME%\Scripts;%PATH%
set PYTHONPATH=

:: install ROS system dependencies
rosdep init
rosdep update
rosdep install --from-paths c:\opt\ros\%ROS_DISTRO%\x64\share --ignore-src -r -y
```
