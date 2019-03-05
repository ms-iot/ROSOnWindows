# Setup `rosserial_arduino` with ROS on Windows

## prerequisite

- ROS on Windows installation: https://github.com/ms-iot/ROSOnWindows/blob/master/GettingStarted/Setup.md
- Arduino: https://www.arduino.cc/en/Main/Software
    - *note: choose Windows Installer (instead of Windows app) to use Arduino with VS Code*
- *optional:* VS Code: https://code.visualstudio.com/
- *optional:* VS Code Arduino plugin: https://marketplace.visualstudio.com/items?itemName=vsciot-vscode.vscode-arduino

## Install

1. follow [ROS on Windows setup instructions](https://github.com/ms-iot/ROSOnWindows/blob/master/GettingStarted/Setup.md) to install ROS on Windows binaries (install from source code works too, but might be more complicated)

2. (if ROS on Windows is already installed) use the following `choco upgrade` command to make sure you have the latest binaries
    ```
    choco upgrade ros-melodic-desktop -y
    ```

3. load ROS on Windows environment with `setup.bat`, similar to the following command
    ```
    C:\opt\ros\melodic\x64\setup.bat
    ```

    *note: this step is critical; otherwise all the ROS CLI tools like `catkin_make` or `catkin_make_isolated` cannot be used*

4. before moving on to `rosserial` installation, check environment
    - you should see this is ROS environment is loaded
        ```
        >set ros
        ROS_DISTRO=melodic
        ROS_ETC_DIR=C:/opt/ros/melodic/x64/etc/ros
        ROS_MASTER_URI=http://localhost:11311
        ROS_PACKAGE_PATH=C:\opt\ros\melodic\x64\share
        ROS_PYTHON_VERSION=2
        ROS_ROOT=C:/opt/ros/melodic/x64/share/ros
        ROS_VERSION=1
        >set path
        Path=C:/opt/rosdeps/x64\bin;C:/opt/rosdeps/x64\lib;C:/opt/ros/melodic/x64\bin;C:/opt/ros/melodic/x64\lib;...
        ```
    - you should see this if Python (for ROS) is added to PATH
        ```
        >set path
        Path=...;C:\opt\python27amd64\;C:\opt\python27amd64\Scripts;...
        ```
    - you should be able to use call Python from command line
        ```
        >python --version
        Python 2.7.15
        ```

5. create `rosserial` workspace, clone `rosserial`, and build

    *note: please use `ms-iot:init_windows` on Windows until `ms-iot` changes are upstreamed back to ROS mainline*
    ```
    mkdir c:\ros\catkin_ws\rosserial\src\ && cd c:\ros\catkin_ws\rosserial\src\
    git clone https://github.com/ms-iot/rosserial.git -b init_windows
    cd ..
    catkin_make_isolated
    ```

6. load `rosserial` environment just have just been built, and setup Arduino environment (assuming current work directory is still `c:\ros\catkin_ws\rosserial`)

    *note: these steps are very similar to [the steps (for Ubuntu) in `rosserial_arduino` tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)*
    ```
    .\install_isolated\setup.bat
    rosrun rosserial_arduino make_libraries.py .
    ```

7. at this point, a new folder called `ros_lib` should have been generated, copy that to the library folder for Arduino
    - library folder for Arduino could be found through these steps:
        1. launch Arduino
        2. Open File\Preferences
        3. find Sketchbook location, the path would be `<path>\Arduino`
        4. Arduino's library path would be `<path>\Arduino\libraries`

8. check if `ros_lib` is installed correctly for Arduino
    1. launch Arduino (close and relaunch if needed)
    2. check File\Examples
    3. there sould be `ros_lib` under `Examples from Custom Libraries`

9. explore `rosserial` by following [its tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials)

    *note: certain `#define` flags need to be added for specific platforms, check [rosserial_arduino](http://wiki.ros.org/rosserial_arduino) for details*
    - especially, add `#define USE_USBCON` for Arduino based on 32u4: Leonardo, Micro
