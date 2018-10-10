# ROS for Windows Porting Cookbook
While every effort has been made to reduce the effort needed to support ROS nodes on Windows, 
there will inevitably be required changes between platforms. This cookbook is intended to collect common issues and recommended solutions.

# Windows vs Linux
## $(find ...) idiom to package:// url
In many cases in ROS, `$(find ...)` is used to locate a resource at runtime. However, the semantics are different on Windows and Linux. 
To alleviate this, please use `package://<ros package id>/<resource>`   

## Paths and ROS commands
Many ROS Commands are sensitive to the drive letter they are executed from. This manifests in problems such as rosdeps not resolving correctly. 

To address this either:
  * Put all of your ROS workspaces on the C:\ drive
  * Link folders from your C:\ drive to your workspaces.

To link a folder on Windows, use the mklink to create a filesystem link from one drive to another.:
``` 
mkdir d:\workspaces
mklink c:\workspaces d:\workspaces
```

## Beware of aggressive optimization
The Microsoft compiler will optimize agressively. This can manifest in strange ways. One instance was in turtlebot3 fake code, is a ROS_ASSERT with a function that only returns true. Nothing else executed.

## Case sensitivity
Linux is case sensitive, whereas Windows is not. We are trying to locate case sensitive areas and isolate them. This manifests in odd errors like this: 

```
RLException: multiple files named [turtlebot3_robot.launch] in package [turtlebot3_bringup]:
- C:\ws\turtlebot_ws\install\share\turtlebot3_bringup\launch\turtlebot3_robot.launch
- c:\ws\turtlebot_ws\install\share\turtlebot3_bringup\launch\turtlebot3_robot.launch
```
In this case, the ROS_PACKAGE_PATH has a lower case drive letter.

# Python
## Shebang
Windows does not support Shebang character sequence for automatically launching an interpreter. To support Python nodes on Windows, a few changes need to be made.
### Shebang in ROS nodes
If a ROS node uses Python, please rename the file with the .py extension.

### Shebang in command line commands
If you are producing a command line application which will be installed with Pip, please add a windows wrapper.


# Errors
## gtest-NOTFOUND
This occurs when linking against gtest instead of ${GTEST_LIBRARIES}
```
  target_link_libraries( rtest
      ${GTEST_LIBRARIES}
      ${catkin_LIBRARIES}
  )
```

## Boost::asio Winsock.h has already been included

### Cause:
ROS includes Windows.h, but explicitly excludes Winsock.h. Boost's socket_types.h checks for this flag and assumes winsock.h was included.

### Fix:
Add the following before boost/asio.hpp:
``` C++
#include <ros/ros.h>

#ifdef WIN32
#include <winsock2.h>
#endif

#include <boost/asio.hpp>
```

# Missing Symbols
## 'M_PI'
Add the following to the top of your file

``` C++
#define _USE_MATH_DEFINES 
```

# Warnings
The Microsoft Visual Studio compiler has strict type checking enabled by default. Here are some common warnings.

## Truncation
'=': truncation from 'double' to float
Use appropriate casts ensuring accuracy of the conversion.

## unreferenced parameters
Either remove it, or reference it in a noop block

```c++ 
uint8_t unused;
unused;
```

##  
