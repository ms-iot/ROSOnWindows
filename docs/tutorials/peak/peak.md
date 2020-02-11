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

## Step 1: Create a ROS package

Firstly, you will need a `catkin` workspace to begin with.
Assuming you have an empty workspace under `c:\can_ws`, now create a new package.

```Batchfile
:: change the directory to the source subfolder.
c:\can_ws> cd src

:: create your owned package
c:\can_ws\src> catkin_create_pkg my_pkg
```

## Step 2: Add PCAN-Basic SDK

The next step is to add `PCAN-Basic` library into your package.
This example shows a way to pull and place the library into the correct location by `CMake` convention.
Now open and edit the `CMakeLists.txt` under `src\my_pkg`.

```CMake
set(TARGET_ARCH "Win32")
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(TARGET_ARCH "x64")
endif()

file(MAKE_DIRECTORY
    ${CMAKE_BINARY_DIR}/installed/pcan-basic/bin
    ${CMAKE_BINARY_DIR}/installed/pcan-basic/lib)
    ${CMAKE_BINARY_DIR}/installed/pcan-basic/include)

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
set(pcan_LIBRARIES ${CMAKE_BINARY_DIR}/installed/pcan-basic/lib/PCANBasic.lib)
set(pcan_INCLUDE_DIRS ${CMAKE_BINARY_DIR}/installed/pcan-basic/include)

install(FILES
    ${CMAKE_BINARY_DIR}/installed/pcan-basic/bin/PCANBasic.dll
    DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
```



[can]: https://en.wikipedia.org/wiki/CAN_bus
[peak]: https://www.peak-system.com/?&L=1
