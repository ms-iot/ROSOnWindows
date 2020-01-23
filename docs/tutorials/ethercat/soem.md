Getting Started with SOEM Application
==============================================

[`EtherCAT` (Ethernet for Control Automation Technology)](https://www.ethercat.org/) is an Ethernet-based fieldbus system.
It is a widely used protocol for industrial applications.
In this tutorial, I will show you how to install and consume an open source `EtherCAT` master implementation, `SOEM` (Simple Open EtherCAT Master), and discover devices on a `EtherCAT` network in a `ROS` workspace.

For more information, visit [https://github.com/OpenEtherCATsociety/SOEM](https://github.com/OpenEtherCATsociety/SOEM).

## Prerequisite

  * You have a machine with `Windows 10` installed.
  * You have [`ROS Melodic Desktop Full`](https://wiki.ros.org/Installation/Windows) installed.
  * You have a `EtherCAT` slave controller commissioned to work.
  * You have a `EtherCAT` network topology configured physically.
  * You have basic knowledge to `EtherCAT` technology.

## Step 1: Add SOEM Library to your Workspace

Firstly, you will need a `catkin` workspace to begin with.
Let's assume you are working on an empty workspace under `c:\ethercat_ws`.
You can install `SOEM` from `vcpkg`:

```no-highlight
:: install vcpkg SOEM port
c:\ethercat_ws> vcpkg install SOEM:x64-windows
```

## Step 2: Create your own Package

Now you have the required libraries.
Let's begin with a new package and build with `SOEM`.

```no-highlight
:: change the directory to the source subfolder.
c:\ethercat_ws> cd src

:: create your owned package
c:\ethercat_ws\src> catkin_create_pkg my_pkg
```

## Step 3: Add SOEM Application Code

Copy [`slaveinfo.c`](https://raw.githubusercontent.com/OpenEtherCATsociety/SOEM/master/test/linux/slaveinfo/slaveinfo.c) into the new package as the `SOEM` application code to run.

```no-highlight
:: make the directory to store the source.
c:\ethercat_ws> mkdir src\my_pkg\src

:: download the source of `slaveinfo.c`
c:\ethercat_ws> curl https://raw.githubusercontent.com/OpenEtherCATsociety/SOEM/master/test/linux/slaveinfo/slaveinfo.c -o src\my_pkg\src\slaveinfo.c
```

## Step 4: Edit CMakeLists.txt

You will also needs to edit the `src\my_pkg\CMakeLists.txt` to author the new executable and to be explicit on `SOEM` as dependency.

Here is the `CMake` recipe to add to look for `SOEM`:

```cmake
find_path(winpcap_INCLUDE_DIRS NAMES pcap.h)
find_library(winpcap_LIBRARY NAMES wpcap)
find_library(packet_LIBRARY NAMES packet)
find_path(soem_INCLUDE_DIRS NAMES ethercat.h PATH_SUFFIXES soem)
find_library(soem_LIBRARY NAMES soem)
```

Here is the recipe to author the `slaveinfo.c` as an excutable:

```cmake
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${soem_INCLUDE_DIRS}
  ${winpcap_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/slaveinfo.c)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME slaveinfo PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${soem_LIBRARY}
  ${winpcap_LIBRARY}
  ${packet_LIBRARY}
  Ws2_32.lib Winmm.lib
)
```

## Step 5: Build the Workspace

Let's build the workspace to produce the executables and binaries.
And remember to run `setup.bat` to add the development space into the current environment.

```no-highlight
:: Build the workspace
c:\ethercat_ws> catkin_make

:: Add the development space into the current environment.
c:\ethercat_ws> devel\setup.bat
```

## Step 6: Check EtherCAT Network

Let's check the `EtherCAT` network to make sure the `SOEM` and the `EtherCAT` network is working.

First, retrieve the list of ethernet adapter discovered on your machine.

```no-highlight
c:\ethercat_ws> rosrun my_pkg slaveinfo

SOEM (Simple Open EtherCAT Master)
Slaveinfo
Usage: slaveinfo ifname [options]
ifname = eth0 for example
Options :
 -sdo : print SDO info
 -map : print mapping
Available adapters
Description : Microsoft, Device to use for wpcap: \Device\NPF_{7B78D297-ED15-4C1B-BC32-8EDB9E41E5AB}
Description : VMware Virtual Ethernet Adapter, Device to use for wpcap: \Device\NPF_{8BBA06A7-8AAB-48B5-90E3-DC2DB31D86F6}
Description : Microsoft, Device to use for wpcap: \Device\NPF_{BF10C5FD-EF74-4D93-B32E-D0D2A040BA87}
Description : Microsoft Corporation, Device to use for wpcap: \Device\NPF_{894B5E76-A359-4AAF-8F7C-41FAD68773E3}
Description : Microsoft, Device to use for wpcap: \Device\NPF_{A54146EA-CD6F-4A95-93E4-0919C4B2D685}
Description : VMware Virtual Ethernet Adapter, Device to use for wpcap: \Device\NPF_{144028BF-951B-4209-B8D4-30E888BFB4CB}
End program
```

Look for the ethernet adapter for your `EtherCAT` network and take a note of the path, for example, `\Device\NPF_{A54146EA-CD6F-4A95-93E4-0919C4B2D685}`.

Now run the `slaveinfo` with the ethernet device path again to list what `EtherCAT` slave controllers are discovered on the network.
You should see information similar to the below and check that the details make sense to your environment.

```no-highlight
c:\ethercat_ws> rosrun soem slaveinfo \Device\NPF_{A54146EA-CD6F-4A95-93E4-0919C4B2D685}

Slave:1
 Name:simco drive 40028083-00-0
 Output size: 48bits
 Input size: 48bits
 State: 4
 Delay: 0[ns]
 Has DC: 0
 Activeports:1.1.0.0
 Configured address: 1002
 Man: 0000010a ID: 0262c7b3 Rev: 00010211
 SM0 A:1000 L: 512 F:00010026 Type:1
 SM1 A:1400 L: 512 F:00010022 Type:2
 SM2 A:1800 L:   6 F:00010064 Type:3
 SM3 A:1950 L:   6 F:00010020 Type:4
 FMMU0 Ls:0000000a Ll:   6 Lsb:0 Leb:7 Ps:1800 Psb:0 Ty:02 Act:01
 FMMU1 Ls:00000027 Ll:   6 Lsb:0 Leb:7 Ps:1950 Psb:0 Ty:01 Act:01
 FMMUfunc 0:1 1:2 2:0 3:0
 MBX length wr: 512 rd: 512 MBX protocols : 0c
 CoE details: 27 FoE details: 01 EoE details: 00 SoE details: 00
 Ebus current: 0[mA]
 only LRD/LWR:0
PDO mapping according to CoE :
  SM2 outputs
     addr b   index: sub bitl data_type    name
  [0x000A.0] 0x6040:0x00 0x10 UNSIGNED16   Controlword
  [0x000C.0] 0x60FF:0x00 0x20 INTEGER32    Target Velocity
  [0x0010.0] 0x0000:0x00 0x00
  [0x0010.0] 0x0000:0x00 0x00
  [0x0010.0] 0x0000:0x00 0x00
  [0x0010.0] 0x0000:0x00 0x00
  [0x0010.0] 0x0000:0x00 0x00
  [0x0010.0] 0x0000:0x00 0x00
  SM3 inputs
     addr b   index: sub bitl data_type    name
  [0x0027.0] 0x6041:0x00 0x10 UNSIGNED16   Statusword
  [0x0029.0] 0x6064:0x00 0x20 INTEGER32    Position Actual Value
  [0x002D.0] 0x0000:0x00 0x00
  [0x002D.0] 0x0000:0x00 0x00
  [0x002D.0] 0x0000:0x00 0x00
  [0x002D.0] 0x0000:0x00 0x00
  [0x002D.0] 0x0000:0x00 0x00
  [0x002D.0] 0x0000:0x00 0x00
```

# Summary

Now you have a workspace to continue the learning of `SOEM`.
In this tutorial, I walk through the steps of installing `SOEM` from `vcpkg`, how to edit catkin package to consume `SOEM`, and how to run `slaveinfo` to exercise your EtherCAT network.
I encourge you to read on the official `SOEM` [tutorial](https://openethercatsociety.github.io/doc/soem/tutorial_8txt.html) to learn more about how to program `SOEM` application.
