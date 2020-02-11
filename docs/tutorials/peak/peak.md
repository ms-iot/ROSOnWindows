---
title: Using PEAK-System SDK on Windows
---

[Controller Area Network (CAN) Bus][can] is one of the widely used field bus protocols in industrial automation.
In this tutorial, we will show you an example how to begin with a ROS package, integrate with a CAN adapter device on PC, and perform the basic I/O with your CAN Bus.

In this example, we will be using PEAK-System PCAN-Basic API. For more details, visit [PEAK-System][peak].

## Prerequisite
  * You have a machine with `Windows 10` installed.
  * You have [`ROS Melodic Desktop Full`](https://wiki.ros.org/Installation/Windows) installed.
  * You have PEAK-Basic supported CAN adapters.
    We use `PCAN-miniPCIe` in this tutorial.
  * You have the CAN adapters connected to a CAN network and all devices are commissioned to work.
  * You have a ROS command prompt ready to use.

## Step 1: Creating a ROS package

Firstly, you will need a `catkin` workspace to begin with.
Assuming you have an empty workspace under `c:\can_ws`, now create a new package.

```Batchfile
:: change the directory to the source subfolder.
c:\can_ws> cd src

:: create your owned package
c:\can_ws\src> catkin_create_pkg my_pkg
```

## Step 2: Adding PCAN-Basic SDK

The next step is to add `PCAN-Basic` library into your package.
This example shows a way to pull and place the library into the correct location by `CMake` convention.
Now open and edit the `CMakeLists.txt` under `src\my_pkg`.

```CMake
# Decide what architecture for the target
set(TARGET_ARCH "Win32")
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(TARGET_ARCH "x64")
endif()

# Create locations to place the PCAN-Basic library
file(MAKE_DIRECTORY
    ${CMAKE_BINARY_DIR}/installed/pcan-basic/bin
    ${CMAKE_BINARY_DIR}/installed/pcan-basic/lib
    ${CMAKE_BINARY_DIR}/installed/pcan-basic/include)

# Pull down the PCAN-Basic library
include(ExternalProject)
ExternalProject_Add(
    pcan-basic
    URL https://www.peak-system.com/fileadmin/media/files/pcan-basic.zip
    URL_MD5 d388e723046e7b2f6bd06489a12a3d96
    PREFIX ${CMAKE_BINARY_DIR}/pcan-basic
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory
                    <SOURCE_DIR>/include ${CMAKE_BINARY_DIR}/installed/pcan-basic/include
    COMMAND ${CMAKE_COMMAND} -E copy
            <SOURCE_DIR>/${TARGET_ARCH}/PCANBasic.dll ${CMAKE_BINARY_DIR}/installed/pcan-basic/bin
    COMMAND ${CMAKE_COMMAND} -E copy
            <SOURCE_DIR>/${TARGET_ARCH}/VC_LIB/PCANBasic.lib ${CMAKE_BINARY_DIR}/installed/pcan-basic/lib
    LOG_DOWNLOAD ON
    LOG_INSTALL ON
    )

# Initialize include paths and library paths
set(pcan_LIBRARIES ${CMAKE_BINARY_DIR}/installed/pcan-basic/lib/PCANBasic.lib)
set(pcan_INCLUDE_DIRS ${CMAKE_BINARY_DIR}/installed/pcan-basic/include)

# Remember to install the DLL side-by-side to ROS application
install(FILES
    ${CMAKE_BINARY_DIR}/installed/pcan-basic/bin/PCANBasic.dll
    DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
```

And now you can build the package and see if everything is set up correctly.

```Batchfile
:: Build the workspace
c:\can_ws> catkin_make install

:: Add the install space into the current environment.
c:\can_ws> install\setup.bat
```

## Step 3: Adding Simple CAN Read\Write Loop

Now you have the PCAN-Basic library ready to use in your ROS workspace.
We are going to add a simple node doing the basic read/write loop.
Now you go to the editor, create a file of `src/my_pkg/src/my_pkg_node.cpp` under your workspace, and copy & paste the below code:

```c++
#include <windows.h>
#include <PCANBasic.h>

#include <exception>
#include <iostream>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/console.h>

typedef boost::chrono::steady_clock time_source;

class PCANLoop
{
public:
    PCANLoop()
    {
        TPCANStatus result = CAN_Initialize(PCAN_PCIBUS1, PCAN_BAUD_500K);
        if (result != PCAN_ERROR_OK)
        {
            throw std::runtime_error("CAN_Initialize failed.");
        }
    }

    void process()
    {
        send();

        while(read()); // process till the queue goes empty
    }

private:
    bool read()
    {
        TPCANMsg received = {0};

        TPCANStatus result = CAN_Read(PCAN_PCIBUS1, &received, NULL);
        if (result == PCAN_ERROR_QRCVEMPTY)
        {
            return false;
        }
        else if (result == PCAN_ERROR_OK)
        {
            // COMMENT: read the message here.
            return true;
        }

        throw std::runtime_error("CAN_Read failed.");
    }

    void send()
    {
        // COMMENT: replace the message with yours.
        TPCANMsg request = {0};
        request.ID = 0;
        request.MSGTYPE = PCAN_MESSAGE_STANDARD;
        request.LEN = 8;

        TPCANStatus result = CAN_Write(PCAN_PCIBUS1, &request);
        if (result != PCAN_ERROR_OK)
        {
            throw std::runtime_error("CAN_Write failed.");
        }
    }
};

/**
* Control loop for PCAN
*/
void controlLoop(PCANLoop &loop)
{
    try
    {
        loop.process();
    }
    catch (std::exception &e)
    {
        ROS_ERROR_STREAM("controlLoop exception:" << e.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh, private_nh("~");

    try
    {
        double control_frequency;
        private_nh.param<double>("control_frequency", control_frequency, 50.0);

        PCANLoop loop;

        ros::CallbackQueue queue;
        ros::AsyncSpinner spinner(1, &queue);

        ros::TimerOptions control_timer(
            ros::Duration(1 / control_frequency),
            boost::bind(controlLoop, boost::ref(loop)),
            &queue);
        ros::Timer control_loop = nh.createTimer(control_timer);

        spinner.start();

        // Process remainder of ROS callbacks separately, mainly ControlManager related
        ros::spin();
    }
    catch (std::exception &e)
    {
        ROS_ERROR_STREAM("exception:" << e.what());
    }

    return 0;
}
```

We will explain the code later.
We need to describe the new node and its dependency in `CMakeLists.txt` and `package.xml`.
Now open and edit the `src\my_pkg\CMakeLists.txt` file under the workspace.

### `src\my_pkg\CMakeLists.txt`

Below is an example to describe a new node in `CMake`.

```CMake

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp)

...

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${pcan_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/my_pkg_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${pcan_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### `src\my_pkg\package.xml`

Make sure `roscpp` in the `<depend>` list.

```xml
<?xml version="1.0"?>
<package format="2">
  <name>my_pkg</name>
  ...
  <depend>roscpp</depend>
  ...
</package>
```

### Simple CAN Read/Write Loop

In this example, we use [`Callback and Spinning`][callback-spinning] from `roscpp` to set up a control loop running at 50Hz.
In the callback, we read all the messages from the PCAN message queue and send a empty message to the CAN Bus.
Depending on your CAN devices, you may need to add more protocol-specific implementation on top of the basic I/O.

## Step 4: Building the Workspace

Now we have all the code in place.
Let's rebuild the workspace again to ensure everything built.

```Batchfile
:: Build the workspace
c:\can_ws> catkin_make install

:: Add the install space into the current environment.
c:\can_ws> install\setup.bat
```

## Step 5: Running the ROS Application

Before we are ready to launch the application, we need to make sure `rosmaster` is up and running.
Start another ROS command prompt and run `roscore`.
Now we are ready to run this application.

```Batchfile
c:\can_ws> rosrun my_pkg node
```

## Summary

In this tutorial, I walk through the steps of integrate `PCAN-Basic` library into your ROS package, how to consume it by an simple CAN read/write loop application.
I encourge you to proceed on the official `PCAN-Basic` [documentation][pcan-basic] to learn more.



[can]: https://en.wikipedia.org/wiki/CAN_bus
[peak]: https://www.peak-system.com/?&L=1
[pcan-basic]: https://www.peak-system.com/PCAN-Basic.239.0.html?&L=1
[callback-spinning]: http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
