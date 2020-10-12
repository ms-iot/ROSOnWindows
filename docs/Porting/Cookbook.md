# ROS for Windows Porting Cookbook
While every effort has been made to reduce the effort needed to support ROS nodes on Windows, 
there will inevitably be required changes between platforms. This cookbook is intended to collect common issues and recommended solutions.
## Windows and Linux Differences
### Directory Separators
Windows uses backslash `\` whereas Linux uses forward slash `/`. As we encounter path processing, we've been replacing them with the Python or Boost equivelents.

### User directory
Linux has a neat shortcut for refering to the users' home directory `~`. 

Windows uses the environment variable `%USERPROFILE%` - use this whenever you see `~` in ROS documentation.

## Quote handling in command window
Cmd.exe is the command processor of command window.  Single quotes are not used at all by the cmd.exe except in batch file to enclose the command to run within a FOR /F statement.  Cmd.exe handles quoting with double quotes.  This is different from Linux that uses single quote as quote character.  As encounter quoting on Windows, please use double quote.  The following example shows using double quotes around the message contents:
```bat
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- "[2.0, 0.0, 0.0]" "[0.0, 0.0, 1.8]"
```

### Paths and ROS commands
Many ROS Commands are sensitive to the drive letter they are executed from. This manifests in problems such as rosdeps not resolving correctly. 

To address this either:
  * Put all of your ROS workspaces on the C:\ drive
  * Link folders from your C:\ drive to your workspaces.

To link a folder on Windows, use the mklink to create a filesystem link from one drive to another.:
```bat
mkdir d:\workspaces
mklink c:\workspaces d:\workspaces
```

### Symbol Visibility
Windows and Linux handle symbol visibility differently. You may encounter a build error of the form:
```bat
error C2448: '__attribute__': function-style initializer appears to be a function definition
'visibility': identifier not found
```

[The GCC visibility documentation](https://gcc.gnu.org/wiki/Visibility)&nearr; has a good article on handling the compiler differences. This includes adding a header file which uses a compiler switch to select the correct visibility macro handler.

Symbol Visibility also impacts binary loading. If you are finding a Nodelet that does not run or a Qt Visualizer isn't working, it may be that the hosting process can not find an expected binary export. To diagnose this on Windows, the Windows developer tools includes a program called Gflags to enable various options. One of those options is called *Loader Snaps* which enables you to detect load failures while debugging. 

Please visit the Microsoft Documentation for more information on [Gflags](https://docs.microsoft.com/en-us/windows-hardware/drivers/debugger/setting-and-clearing-image-file-flags) and  [Loaders snaps](https://docs.microsoft.com/en-us/windows-hardware/drivers/debugger/show-loader-snaps).

### install Library TARGETS given no DESTINATION! 

Windows will generate separate archives and librarys. To handle this, add an ARCHIVE destination:
```bat
install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
```
### All Warnings
Warnings are good. The options for selecting warning level are different. If you add specific compiler options for warnings, please add an MSVC selection. For the Visual Studio compiler, you'll use `/W3` for warning level 3 (or `/W4` which offers more warning options). If you would like to treat warnings as errors pass `/WX`. However, these warnings would need to be corrected before the compile will succeed.

```bat
if(MSVC)
  add_compile_options(/W3 /WX)
else()
  add_compile_options(-Wall -Wextra)
endif()
```

You can disable specific warnings using `#pragma`:

```c
#ifdef _MSC_VER
  #pragma warning(disable: 4244)
  #pragma warning(disable: 4661)
#endif
```
### Security Warnings
Windows deprecates certain C APIs because they are inherently insecure. You will receive a message of the form:

```bat
warning C4996: 'xxx': This function or variable may be unsafe. Consider using xxx_s instead. To disable deprecation, use _CRT_SECURE_NO_WARNINGS. See online help for details.
```

Consider using modern equivelents. If you cannot use a modern equivelent, you can add the following to your cmake files:

```bat
add_definitions("/D_CRT_SECURE_NO_WARNINGS")
add_definitions("/D_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS")
```

### C++ versioning
Use CMake to set the C++ version:

```bat
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()
```
or 
```bat
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
```

> NOTE: Boost 1.66 in the Melodic requires CMAKE_CXX_STANDARD 11

### `__attribute____`
`__attribute__` is not suppported with Microsoft compilers. You can use a macro replacement or use a cross platform convention.

### Unresolved External
Linux automatically exports symbols. Windows, symbols are private by default. [CMake provides a facility for this](https://cmake.org/cmake/help/v3.4/prop_tgt/WINDOWS_EXPORT_ALL_SYMBOLS.html).

In your cmake:

```bat
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
```

However, you may encounter a problem with static class members, i.e. global data symbols, as discussed in https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/. In this case you need to manually handle visibility of the static class members, using the [`GenerateExportHeader`](https://cmake.org/cmake/help/v3.4/module/GenerateExportHeader.html) CMake module. 

In particular, if your static class members are contained in a library called `mylibrary`, you need to add the following lines in your CMake after the call to
`add_library(mylibrary ...)`: 
~~~
include(GenerateExportHeader)
generate_export_header(mylibrary)
target_include_directories(mylibrary PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
~~~

The `generate_export_header(mylibrary)` call creates a file called `mylibrary_export.h`, contained in the `${CMAKE_CURRENT_BINARY_DIR}` build directory. This file contains the definition of the `mylibrary_EXPORT` macro, that you can use to modify your code as in the following: 
~~~
#include "mylibrary_export.h"

class myclass
{
 static MYLIBRARY_EXPORT int GlobalCounter;
…
~~~

If this class definition is contained in a public header, you need to make sure to install the generated 
`mylibrary_export.h` together with the rest of the headers, i.e. : 
~~~
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/mylibrary_export.h
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
~~~


### Beware of aggressive optimization
The Microsoft compiler will optimize agressively. This can manifest in strange ways. One instance was in turtlebot3 fake code, is a ROS_ASSERT with a function that only returns true. Nothing else executed.

### Case sensitivity
Linux is case sensitive, whereas Windows is not. We are trying to locate case sensitive areas and isolate them. This manifests in odd errors like this: 

```bat
RLException: multiple files named [turtlebot3_robot.launch] in package [turtlebot3_bringup]:
- C:\ws\turtlebot_ws\install\share\turtlebot3_bringup\launch\turtlebot3_robot.launch
- c:\ws\turtlebot_ws\install\share\turtlebot3_bringup\launch\turtlebot3_robot.launch
```
In this case, the ROS_PACKAGE_PATH has a lower case drive letter.

## Python
### Shebang
Windows does not support Shebang character sequence for automatically launching an interpreter. To support Python nodes on Windows, a few changes need to be made.
#### Shebang in ROS nodes
If a ROS node uses Python, please rename the file with the .py extension.

#### Shebang in command line commands
If you are producing a command line application which will be installed with Pip, please add a windows wrapper.


## Errors
### gtest-NOTFOUND
This occurs when linking against gtest instead of ${GTEST_LIBRARIES}
```bat
  target_link_libraries( rtest
      ${GTEST_LIBRARIES}
      ${catkin_LIBRARIES}
  )
```

### Syntax error: 'constant' (windows.h included before conflicting dependency)

#### Cause:

`Windows.h` defines some macros which may conflict with variables defined in ROS packages. This causes build breaks if `windows.h` is included before a conflicting dependency.

The error for this may be misleading as it may not identify the specific redeclaration as the issue. 
#### Example:
C:\ws_moveit\src\moveit_tutorials\doc\perception_pipeline\src\cylinder_segment.cpp

In this file in the **MoveIt** tutorials, a file which included the `window.h` header (pcl) was included before a file which included (marker.h via planning_scene_interface.h).
This is problematic because marker.h defines `DELETE` as an enum member while `DELETE` is also defined as a macro in `windows.h`.

In this case the build error was:
``` Batchfile
C:\opt\ros\melodic\x64\include\visualization_msgs/Marker.h(132): error C2059: syntax error: 'constant'
```
#### Fix:

Include any headers which define the conflicting variable (i.e. `DELETE`) above/before any headers which include `windows.h`.


## Missing Symbols
### 'M_PI'
Add the following to the top of your file:

``` C++
#define _USE_MATH_DEFINES
```

or define it in the CMakeFile.exe

## Warnings
The Microsoft Visual Studio compiler has strict type checking enabled by default. Here are some common warnings.

### Truncation
'=': truncation from 'double' to float
Use appropriate casts ensuring accuracy of the conversion.

### unreferenced parameters
Either remove the variable, or reference it in a noop block

```c++
uint8_t unused;
unused;
```

## CMAKE_C_COMPILER or CMAKE_CXX_COMPILER error
When Visual Studio upgrades, it changes the path to the compilers. If you have previously built a ROS workspace, you'll see an error like this:
```
CMake Error in CMakeLists.txt:
  The CMAKE_CXX_COMPILER:

    C:/Program Files (x86)/Microsoft Visual Studio/2019/Community/VC/Tools/MSVC/14.22.27905/bin/Hostx64/x64/cl.exe
```
### Cause
This is caused by a stale CMake cache.

### Fix
Remove the build, devel and install directories and rebuild.

## CMake Tip - verbose output
When debugging a build failure, it is sometimes helpful to have verbose logging enabled:

```
catkin_make -DCMAKE_VERBOSE_MAKEFILE=ON
```


## Start ROS on Boot
Once you are done developing your robot and want it to automatically start on boot, you'll want to use the [Windows Task Scheduler][taskschd] to start the task.

* Create a Windows command file, which includes the ROS environment and Install environment,

`c:\catkin_ws\start_ros.bat`
``` Batchfile
call c:\opt\ros\melodic\x64\setup.bat
call c:\catkin_ws\install\setup.bat
roslaunch <package> <launch file>
```
* Use the command [Schtasks](https://docs.microsoft.com/en-us/windows/win32/taskschd/schtasks), to add a task which calls this script:
``` Batchfile
schtasks /Create /RU <User> /RP <password> /SC ONLOGON /TN ROS /TR "c:\catkin_ws\start_ros.bat"
```

* Enable Windows Auto logon
Use the [Autologon documentation at Microsoft's Support website](https://support.microsoft.com/en-us/help/324737/how-to-turn-on-automatic-logon-in-windows) to conigure autologon.

Alternatively use the [Autologon Sysinternals tool](https://docs.microsoft.com/en-us/sysinternals/downloads/autologon) to configure autologon from the command line.

The next time the system starts, the ROS task will run.

[taskschd]: https://docs.microsoft.com/en-us/windows/win32/taskschd
