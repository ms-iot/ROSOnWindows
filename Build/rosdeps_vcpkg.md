# Vcpkg, ROS1 system dependencies, and Azure DevOps CI investigation

In ROS1, there are many open sources project required as a minimum to run ROS desktop stack. In the beginning, we either privately built the libraries by project-specific build instructions or looked for stock binaries if it just works.

This approach works fine for the very initial ROS1 porting to manage the initial set of open sources libraries. However, we are also seeing problems with this method:
* **Build Instruction Management (How-to-build-xyz):** Every new project could come with its owned way to build end-to-end. We should share the knowledge in a managible way.
* **Updatibility:** The usage of stock binaries, it would be a bottleneck for us to upgrade its downstream projects (thinking about ABI changes or compatiblity), unless we know how to build them.
* **Latest Visual C++ toolchain support:** Not every project also keeps its build instructions or code up-to-dated for the latest Visual C++ toolchain. We want the binaries always come from the best of breed.

To address those issues, we are considering a fantastic project - Vcpkg. Vcpkg comes the following features:
* Managing the how-to-build sauces of eight hundred and more open-source projects. And it is still growing!
* With framework (Vcpkg helper) support, users are easily to compile libraries cross-platform and using the latest toolchain.

## ROS1 System Dependencies Inventory List

To eavlaute the effort to switch to Vcpkg, firstly begin with a list of system dependencies currently hosted for ROS1 on Windows:

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
| console_bridge | 0.4.0      | ? |
| libccd | 2.0.0              | ? |
| libcurl | 7.58.0            | ? |
| libfcl | 0.5.0              | ? |
| libjpeg-turbo | 1.5.3       | 1.5.3-1 |
| libopencv | 3.4.1           | ? |
| cppunit | 1.12.1            | ? |
| libpng | 1.6.35             | ? |
| eigen | 3.3.4               | ? |
| libqhull | 2015.2.0         | ? |
| freeglut | 3.0.0            | ? |
| log4cxx | 0.10.0            | ? |
| google-mock | 1.8.0         | ? |
| cairo | 1.15.12             | ? |
| google-test | 1.8.0         | ? |
| gtk2 | 2.22.1               | ? |
| gtk3 | 3.22.19              | ? |
| poco | 1.8.1                | ? |
| pyqt5 | 5.10.1              | ? |
| OpenNI | 1.0.0              | ? |
| OpenNI2 | 1.0.0             | ? |
| clapack | 3.2.1             | ? |
| gflags | 2.2.1              | ? |
| pyside2 | 5.10.1            | ? |
| glog | 0.3.5                | ? |
| openblas | 0.2.20           | ? |
| suitesparse | 1.4.0         | ? |
| metis | 5.1.0               | ? |
| sbcl | 0.0.0                | ? |
| abseil | 2018.11.1          | ? |
| sdl | 1.2.15                | 1.2.15-3 |
| protobuf | 3.6.1            | ? |
| freeimage | 3.17.0          | ? |
| cppzmq | 4.2.2              | 4.2.2-1 |
| zeromq | 4.2.5              | 2018-11-01? |
| dlfcn-win32 | 1.1.1         | 1.1.1-1 |
| libwebp | 0.6.1             | 0.6.1-2 |
| openjpeg | 2.3.0            | 2.3.0 |
| tiff | 4.0.9                | ? |
| ilmbase | 2.2.1             | ? |
| sdl_image | 1.2.12          | No? |
| jxrlib | 1.1.0              | ? |
| sip | 4.19.8                | No |
| tinyxml | 2.6.2             | 2.6.2-2 |
| libraw | 0.19.0             | ? |
| tinyxml2 | 6.1.0            | 6.2.0 |
| liblzma | 5.2.4             | 5.2.4 |
| urdfdom | 1.0.0             | 1.0.0-2 |
| openexr | 2.2.1             | 2.2.1-1 |
| urdfdom_headers | 1.0.0     | 1.0.0-2 |
| lcms | 2.8.0                | 2.8-5 |
| yaml-cpp | 0.5.3            | 0.6.2-2 |
| jasper | 2.0.14             | 2.0.14-1 |
| ceres | 1.14.0              | 1.14.0-1 |
| qwt | 6.1.3                 | ? |
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
