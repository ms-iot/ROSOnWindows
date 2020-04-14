```batch
@echo off
:: seed ROS_DISTRO for the install scripts.
echo set "ROS_DISTRO=%ROS_DISTRO%" >> tools\setup.bat
echo set "ROS_ETC_DIR=%ROS_ETC_DIR%" >> tools\setup.bat
echo set "PYTHONHOME=%PYTHON_LOCATION%" >> tools\setup.bat
echo set "ROS_PYTHON_VERSION=%ROS_PYTHON_VERSION%" >> tools\setup.bat

:: create Chocolatey packages.
md output
choco pack --trace --out output <ros package>.nuspec 
md output-pre
choco pack --trace --out output-pre <ros package>.nuspec
```
