# [WIP] Vcpkg, ROS1 system dependencies, and Azure DevOps CI integration

In ROS1, there are many open sources project required as a minimum to run ROS desktop stack. In the beginning, we either privately built the libraries by project-specific build instructions or looked for stock binaries if it just works.

This approach works fine for the very initial ROS1 porting to manage the initial set of open sources libraries. However, we are also seeing problems with this method:
* **Build Instruction Management (How-to-build-xyz):** Every new project could come with its owned way to build end-to-end. We should share the knowledge in a managible way.
* **Updatibility:** Using stock binaries, it would be a bottleneck for us to upgrade its downstream projects (thinking about ABI changes or compatiblity), unless we know how to build them.
* **Latest Visual C++ toolchain support:** Not every project keeps its build instructions or code up-to-dated for the latest Visual C++ toolchain. We want the binaries always come from the best of breed.

To address those issues, we are looking for any leverages to solve them. Here Vcpkg comes.

## Vcpkg

"Vcpkg simplifies acquiring and building open source libraries on Windows." - [Visual C++ Team Blog](https://blogs.msdn.microsoft.com/vcblog/2016/09/19/vcpkg-a-tool-to-acquire-and-build-c-open-source-libraries-on-windows/)

It comes with the following features:
* Managing the how-to-build sauces of eight hundred and more open-source projects. And it is still growing!
* Clear visibility on the dependencies relationship between projects. Now we can better manage what to update after an upstream project gets updated.
* Everything is compiled against the same set of depedencies, so no more ABI hazards.

Now let's take a look ROS1 system dependencies for Windows (as of today).

## ROS1 System Dependencies (Target: Melodic Windows 10)

To evalute switching to Vcpkg, firstly we'd like to know anything missing from Vcpkg ports. After comparison, we see most of packages to exist on Vcpkg. Only few don't exist and need some more investigations:
* libompl
* log4cxx
* libgraphviz
* pkg-config

Also, some projects are Python modules mixing C/C++ sources (e.g., pyside2), and Vcpkg currently doesn't manage this type of projects.

## References
* [ROS1 Target Platforms](http://www.ros.org/reps/rep-0003.html#melodic-morenia-may-2018-may-2023)
* Comparsion Table of ROS1 System Dependencies and Vcpkg Ports

| Project Name   | Version | Vcpkg ports? |
|-------|-----|-----|
| libflann | 1.9.1            | 1.9.1-8 |
| zlib | 1.2.11               | 1.2.11-3 |
| octomap | 1.9.0             | cefed0c1d79afafa5aeb05273cf1246b093b771c-3? |
| libglew | 2.1.0             | 2.1.0-1 |
| ogre | 1.10.11              | 1.10.11 |
| libompl | 1.2.2             | No |
| assimp | 4.0.1              | 4.1.0-3 |
| boost | 1.66.0              | 1.68.0 |
| bullet3 | 2.87.0            | 2.87 |
| bzip2 | 1.0.6               | 1.0.6-3 |
| console_bridge | 0.4.0      | 0.3.2-3 |
| libccd | 2.0.0              | 2.0.0-2 |
| libcurl | 7.58.0            | 7.61.1-1 |
| libfcl | 0.5.0              | 0.5.0-3 |
| libjpeg-turbo | 1.5.3       | 1.5.3-1 |
| libopencv | 3.4.1           | 3.4.3-3 |
| cppunit | 1.12.1            | 1.14.0 |
| libpng | 1.6.35             | 1.6.35-1 |
| eigen | 3.3.4               | 3.3.5 |
| libqhull | 2015.2.0         | 2015.2-3 |
| freeglut | 3.0.0            | 3.0.0-5 |
| log4cxx | 0.10.0            | No |
| google-mock | 1.8.0         | 1.8.1-1 (gtest) |
| cairo | 1.15.12             | 1.15.8-4 |
| google-test | 1.8.0         | 1.8.1-1 |
| gtk2 | 2.22.1               | No |
| gtk3 | 3.22.19              | 3.22.19-2 |
| poco | 1.8.1                | 1.9.0-1 |
| pyqt5 | 5.10.1              | No (python) |
| OpenNI | 1.0.0??              | No |
| OpenNI2 | 1.0.0??             | 2.2.0.33-7 |
| clapack | 3.2.1             | 3.2.1-1 |
| gflags | 2.2.1              | 2.2.2-1 |
| pyside2 | 5.10.1            | No (python) |
| glog | 0.3.5                | 0.3.5-1 |
| openblas | 0.2.20           | 0.2.20-2 |
| suitesparse | 1.4.0??         | 5.1.2 |
| metis | 5.1.0               | 5.1.0-2 |
| sdl | 1.2.15                | 1.2.15-3 |
| protobuf | 3.6.1            | 3.6.1-4 |
| freeimage | 3.17.0          | 3.18.0-2 |
| cppzmq | 4.2.2              | 4.2.2-1 |
| zeromq | 4.2.5              | 2018-11-01? |
| dlfcn-win32 | 1.1.1         | 1.1.1-1 |
| libwebp | 0.6.1             | 0.6.1-2 |
| openjpeg | 2.3.0            | 2.3.0 |
| tiff | 4.0.9                | 4.0.10-1 |
| ilmbase | 2.2.1             | 2.2.1-1 |
| sdl_image | 1.2.12          | No? |
| jxrlib | 1.1.0              | 1.1-4 |
| sip | 4.19.8                | No (python) |
| tinyxml | 2.6.2             | 2.6.2-2 |
| libraw | 0.19.0             | 0.19.0-1 |
| tinyxml2 | 6.1.0            | 6.2.0 |
| liblzma | 5.2.4             | 5.2.4 |
| urdfdom | 1.0.0             | 1.0.0-2 |
| openexr | 2.2.1             | 2.2.1-1 |
| urdfdom_headers | 1.0.0     | 1.0.0-2 |
| lcms | 2.8.0                | 2.8-5 |
| yaml-cpp | 0.5.3            | 0.6.2-2 |
| jasper | 2.0.14             | 2.0.14-1 |
| ceres | 1.14.0              | 1.14.0-1 |
| qwt | 6.1.3                 | 6.1.3-6 |
| gazebo9 | 9.4.1             | No |
| libfltk | 1.3.4             | 1.3.4-5 |
| libtbb-dev | 2018.6.0       | 2018_U6 |
| libgraphviz | 2.41.0        | No |
| lz4 | 1.8.1                 | 1.8.3 |
| openssl | 1.1.1             | 1.0.2p-1? |
| orocos_kdl | 1.3.1          | No |
| pkg-config | 0.29.2         | No |
| qt5-sdk | 5.10.1            | 5.11.2 |
| libtheora | 1.1.1           | 1.2.0alpha1-2 |
| libogg | 1.3.3              | 1.3.3 |
| libpcl | 1.8.1              | 1.9.0-1 |
| libazure-iot-sdk-c | 1.2.10 | 1.2.10-1 |
| libqglviewer | 2.7.1        | No |
