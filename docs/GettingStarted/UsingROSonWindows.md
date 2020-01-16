# Using ROS on Windows
While every effort has been made to reduce the effort needed to support ROS on Windows, 
there will inevitably be required changes between platforms. This document is intended to address porting OS differences between Linux and Windows.

Microsoft has created staging forks for many projects. We will be committing back upstream and removing out forks once they are stabilized on Windows and Linux. 
Until these changes are committed upstream, please look at the [Microsoft ms-iot github](https://github.com/ms-iot) to see if there is a fork for the ROS repository you are interested in and use that. Typically, there is also have an init_windows branch which should be used instead of the melodic-devel branch.

# Tips
## Use a Tabbed command line manager
While working with ROS you will need to have multiple command windows open. Each window you open, you need to load the environment. To assist with managing multiple command lines, [ConEmu](https://conemu.github.io/) is a tabbed command line host which works well for managing ROS Command Windows.

## Debugging a ROS node
In many cases, you can launch devenv from within a ROS Command window, then launch the node's executable to debug it. In some cases, you need to debug a node which was launched via ROSLanuch. We've found that the [Visual Studio plugin ReAttach](https://marketplace.visualstudio.com/items?itemName=ErlandR.ReAttach) works well for attaching to a ROS node when it launches.

## ROS for VS Code
If you use Visual Studio Code, then the [ROS for VSCode plugin](https://marketplace.visualstudio.com/items?itemName=ajshort.ros) will be useful. We will be enhancing this for Windows, as well as adding debug options and cloud build connectivity.


# Windows vs Linux
## C++ 17
In your CMakeLists.txt add the compile option:
`add_compile_options(/std:c++latest)`



## Directory Separators
Windows uses backslash `\` whereas Linux uses forward slash `/`. As we encounter path processing, we've been replacing them with the Python or Boost equivelents.

## User directory
Linux has a neat shortcut for refering to the users' home directory `~`. 

Windows - not so much - Please use `%USERPROFILE%` when you see `~` in documentation.

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

## Quote handling in command window
Cmd.exe is the command processor of command window.  Single quotes are not used at all by the cmd.exe except in batch file to enclose the command to run within a FOR /F statement.  Cmd.exe handles quoting with double quotes.  This is different from Linux that uses single quote as quote character.  As encounter quoting on Windows, please use double quote.  The following example shows using double quotes around the message contents:
``` 
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- "[2.0, 0.0, 0.0]" "[0.0, 0.0, 1.8]"
```

## Beware of aggressive optimization
The Microsoft compiler will optimize agressively. This can manifest in strange ways. One instance was in turtlebot3 fake code, is a ROS_ASSERT with a function that only returns true. Nothing else executed.

## Case sensitivity
Linux is case sensitive, whereas Windows is not. We are trying to locate case sensitive areas and fix them. This manifests in odd errors like this: 

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
This occurs when linking against gtest instead of ${GTEST_LIBRARIES}. Update target_link_libraries with the macro:
```
  target_link_libraries( rtest
      ${GTEST_LIBRARIES}
      ${catkin_LIBRARIES}
  )
```

## Boost::asio Winsock.h has already been included
ROS includes Windows.h, but explicitly excludes Winsock.h. Boost's socket_types.h checks for this flag and assumes winsock.h was included.

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
`'=': truncation from 'double' to float`
`'=': truncation from 'size_t' to uint32_t`

It is best to use the same size primitive throughout a codepath. 

## unreferenced parameters
Either remove it, or reference it in a noop block

```c++ 
uint8_t unused;
unused;
```

# Working with Git on Windows

## symbolic links
While symbolic links are natively supported on Linux, they are not fully supported on Windows until around Vista time frame.
Some ROS packages' test cases test against the use of symbolic links, for instance, `roslib` under [`ros\core`](https://github.com/ros/ros)

To work with symbolic links on git repos, check documentation from [Git for Windows](https://github.com/git-for-windows/git/wiki/Symbolic-Links)

##  
