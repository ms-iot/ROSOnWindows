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
* Comparsion Table of ROS1 System Dependencies and vcpkg ports (snap at [2019/09/29](https://github.com/microsoft/vcpkg/tree/94b7f9a6a8c527d7dbdb3b78a46f0874b79712a4))

| package | version | vcpkg port | link | note |
|:-------:|:-------:|:----------:|:----:|:----:|
| libflann | `1.9.1` | `2019-04-07-1` (based on `1.9.1`) | [flann](https://github.com/microsoft/vcpkg/tree/master/ports/flann) | |
| zlib                  | `1.2.11` | `1.2.11-5` | [zlib](https://github.com/microsoft/vcpkg/tree/master/ports/zlib) | |
| octomap               | `1.9.0` | `2017-03-11-7` (based on `1.8.0`) | [octomap](https://github.com/microsoft/vcpkg/tree/master/ports/octomap) | minor version |
| libglew               | `2.1.0` | `2.1.0-6` | [glew](https://github.com/microsoft/vcpkg/tree/master/ports/glew) | |
| ogre                  | `1.10.11` | `1.12.1` | [ogre](https://github.com/microsoft/vcpkg/tree/master/ports/ogre) | minor version |
| libompl               | `1.2.2` | `1.4.2-2` | [ompl](https://github.com/microsoft/vcpkg/tree/master/ports/ompl) | minor version |
| assimp                | `4.0.1` | `5.0.0` | [assimp](https://github.com/microsoft/vcpkg/tree/master/ports/assimp) | **major** version |
| boost                 | `1.66.0` | `1.71.0` | [boost](https://github.com/microsoft/vcpkg/tree/master/ports/boost) | minor version |
| bullet3               | `2.87.0` | `2.88-1` | [bullet3](https://github.com/microsoft/vcpkg/tree/master/ports/bullet3) | minor version |
| bzip2                 | `1.0.6` | `1.0.6-4` | [bzip2](https://github.com/microsoft/vcpkg/tree/master/ports/bzip2) | |
| console_bridge        | `0.4.0` | `0.4.3-1` | [console-bridge](https://github.com/microsoft/vcpkg/tree/master/ports/console-bridge) | |
| libccd                | `2.0.0` | `2.1-1` | [ccd](https://github.com/microsoft/vcpkg/tree/master/ports/ccd) | minor version |
| libcurl               | `7.58.0` | `7.66.0` | [curl](https://github.com/microsoft/vcpkg/tree/master/ports/curl) | minor version |
| libfcl                | `0.5.0` | `0.5.0-6` | [fcl](https://github.com/microsoft/vcpkg/tree/master/ports/fcl) | |
| libjpeg-turbo         | `1.5.3` | `2.0.2` | [libjepg-turbo](https://github.com/microsoft/vcpkg/tree/master/ports/libjpeg-turbo) | **major** version |
| libopencv             | `3.4.1` | `4.1.1-1` | [opencv](https://github.com/microsoft/vcpkg/tree/master/ports/opencv) | **major** version |
| cppunit               | `1.12.1` | `1.14.0` | [cppunit](https://github.com/microsoft/vcpkg/tree/master/ports/cppunit) | minor version |
| libpng                | `1.6.35` | `1.6.37-4` | [libpng](https://github.com/microsoft/vcpkg/tree/master/ports/libpng) | |
| eigen                 | `3.3.4` | `3.3.7-3` | [eigen3](https://github.com/microsoft/vcpkg/tree/master/ports/eigen3) | |
| libqhull              | `2015.2.0` | `7.3.2-1` (based on `2019.1`) | [qhull](https://github.com/microsoft/vcpkg/tree/master/ports/qhull) | **major** version |
| freeglut              | `3.0.0` | `3.0.0-7` | [freeglut](https://github.com/microsoft/vcpkg/tree/master/ports/freeglut) | |
| log4cxx               | `0.10.0` | N/A | | ***missing*** |
| google-mock           | `1.8.0` | `2019-08-14-2` (based on `1.9.0`) | [gtest](https://github.com/microsoft/vcpkg/tree/master/ports/gtest) | minor version |
| cairo                 | `1.15.12` | `1.16.0-2` | [cairo](https://github.com/microsoft/vcpkg/tree/master/ports/cairo) | minor version |
| google-test           | `1.8.0` | `2019-08-14-2` (based on `1.9.0`) | [gtest](https://github.com/microsoft/vcpkg/tree/master/ports/gtest) | minor version |
| gtk2                  | `2.22.1` | N/A | | ***missing*** |
| gtk3                  | `3.22.19` | `3.22.19-3` | [gtk](https://github.com/microsoft/vcpkg/tree/master/ports/gtk) | |
| poco                  | `1.8.1` | `1.9.2-1` | [poco](https://github.com/microsoft/vcpkg/tree/master/ports/poco) | minor version |
| pyqt5                 | `5.10.1` | N/A | | ***Python*** |
| OpenNI                | unsure | N/A | | ***missing*** |
| OpenNI2               | unsure | `2.2.0.33-10` | [openni2](https://github.com/microsoft/vcpkg/tree/master/ports/openni2) | |
| clapack               | `3.2.1` | `3.2.1-12` | [clapack](https://github.com/microsoft/vcpkg/tree/master/ports/clapack) | |
| gflags                | `2.2.1` | `2.2.2-1` | [gflags](https://github.com/microsoft/vcpkg/tree/master/ports/gflags) | |
| pyside2               | `5.10.1` | N/A | | ***Python*** |
| glog                  | `0.3.5` | `0.4.0-2` | [glog](https://github.com/microsoft/vcpkg/tree/master/ports/glog) | minor version |
| openblas              | `0.2.20` | `0.3.6-6` | [openblas](https://github.com/microsoft/vcpkg/tree/master/ports/openblas) | minor version |
| suitesparse           | unsure | `5.4.0-3` | [suitesparse](https://github.com/microsoft/vcpkg/tree/master/ports/suitesparse) | |
| metis                 | `5.1.0` | `5.1.0-5` | [metis](https://github.com/microsoft/vcpkg/tree/master/ports/metis) | |
| sdl                   | `1.2.15` | `1.2.15-8` | [sdl1](https://github.com/microsoft/vcpkg/tree/master/ports/sdl1) | |
| protobuf              | `3.6.1` | `3.9.1` | [protobuf](https://github.com/microsoft/vcpkg/tree/master/ports/protobuf) | minor version |
| freeimage             | `3.17.0` | `3.18.0-7` | [freeimage](https://github.com/microsoft/vcpkg/tree/master/ports/freeimage) | minor version |
| cppzmq                | `4.2.2` | `4.4.1` | [cppzmq](https://github.com/microsoft/vcpkg/tree/master/ports/cppzmq) | minor version |
| zeromq                | `4.2.5` | `2019-09-20` (based on `4.3.3`) | [zeromq](https://github.com/microsoft/vcpkg/tree/master/ports/zeromq) | minor version |
| dlfcn-win32           | `1.1.1` | `1.1.1-3` | [dlfcn-win32](https://github.com/microsoft/vcpkg/tree/master/ports/dlfcn-win32) | |
| libwebp               | `0.6.1` | `1.0.2-7` | [libwebp](https://github.com/microsoft/vcpkg/tree/master/ports/libwebp) | **major** version |
| openjpeg              | `2.3.0` | `2.3.1-1` | [openjpeg](https://github.com/microsoft/vcpkg/tree/master/ports/openjpeg) | |
| tiff                  | `4.0.9` | `4.0.10-7` | [tiff](https://github.com/microsoft/vcpkg/tree/master/ports/tiff) | |
| ilmbase               | `2.2.1` | `2.3.0` | [ilmbase](https://github.com/microsoft/vcpkg/tree/master/ports/ilmbase) | minor version |
| sdl_image             | `1.2.12` | N/A | | ***missing*** |
| jxrlib                | `1.1.0` | `1.1-8` | [jxrlib](https://github.com/microsoft/vcpkg/tree/master/ports/jxrlib) | |
| sip                   | `4.19.8` | N/A | | ***Python*** |
| tinyxml               | `2.6.2` | `2.6.2-4` | [tinyxml](https://github.com/microsoft/vcpkg/tree/master/ports/tinyxml) | |
| libraw                | `0.19.0` | `201903-1` (based on `0.19.0`) | [libraw](https://github.com/microsoft/vcpkg/tree/master/ports/libraw) | |
| tinyxml2              | `6.1.0` | `7.0.1-2` | [tinyxml2](https://github.com/microsoft/vcpkg/tree/master/ports/tinyxml2) | **major** version |
| liblzma               | `5.2.4` | `5.2.4-2` | [liblzma](https://github.com/microsoft/vcpkg/tree/master/ports/liblzma) | |
| urdfdom               | `1.0.0` | `1.0.3-1` | [urdfdom](https://github.com/microsoft/vcpkg/tree/master/ports/urdfdom) | |
| openexr               | `2.2.1` | `2.3.0-4` | [openexr](https://github.com/microsoft/vcpkg/tree/master/ports/openexr) | minor version |
| urdfdom_headers       | `1.0.0` | `1.0.4-1` | [urdfdom-headers](https://github.com/microsoft/vcpkg/tree/master/ports/urdfdom-headers) | |
| lcms                  | `2.8.0` | `2.9` | [lcms](https://github.com/microsoft/vcpkg/tree/master/ports/lcms) | minor version |
| yaml-cpp              | `0.5.3` | `0.6.2-3` | [yaml-cpp](https://github.com/microsoft/vcpkg/tree/master/ports/yaml-cpp) | minor version |
| jasper                | `2.0.14` | `2.0.16-2` | [jasper](https://github.com/microsoft/vcpkg/tree/master/ports/jasper) | |
| ceres                 | `1.14.0` | `1.14.0-6` | [ceres](https://github.com/microsoft/vcpkg/tree/master/ports/ceres) | |
| qwt                   | `6.1.3` | `6.1.3-8` | [qwt](https://github.com/microsoft/vcpkg/tree/master/ports/qwt) | |
| gazebo9               | `9.4.1` | N/A | | ***missing*** |
| libfltk               | `1.3.4` | `1.3.4-7` | [fltk](https://github.com/microsoft/vcpkg/tree/master/ports/fltk) | |
| libtbb-dev            | `2018.6.0` | `2019_U8-1` | [tbb](https://github.com/microsoft/vcpkg/tree/master/ports/tbb) | |
| libgraphviz           | `2.41.0` | N/A | | ***missing*** |
| lz4                   | `1.8.1` | `1.9.2` | [lz4](https://github.com/microsoft/vcpkg/tree/master/ports/lz4) | minor version |
| openssl               | `1.1.1` | `1.0.2s-1` | [openssl-windows](https://github.com/microsoft/vcpkg/tree/master/ports/openssl-windows) | minor version |
| orocos_kdl            | `1.3.1` | `1.4-2` | [orocos-kdl](https://github.com/microsoft/vcpkg/tree/master/ports/orocos-kdl) | minor version |
| pkg-config            | `0.29.2` | N/A | | ***missing*** |
| qt5-sdk               | `5.10.1` | `5.12.5` | [qt5](https://github.com/microsoft/vcpkg/tree/master/ports/qt5) | minor version |
| libtheora             | `1.1.1` | `1.2.0alpha1-20170719~vcpkg1-3` | [libtheora](https://github.com/microsoft/vcpkg/tree/master/ports/libtheora) | minor version |
| libogg                | `1.3.3` | `1.3.4` | [libogg](https://github.com/microsoft/vcpkg/tree/master/ports/libogg) | |
| libpcl                | `1.8.1` | `1.9.1-9` | [pcl](https://github.com/microsoft/vcpkg/tree/master/ports/pcl) | minor version |
| libazure-iot-sdk-c    | `1.2.10` | `2019-08-20.1` (based on `1.3.4`) | [azure-iot-sdk-c](https://github.com/microsoft/vcpkg/tree/master/ports/azure-iot-sdk-c) | minor version |
| libqglviewer          | `2.7.1` | `2.7.0-2` | [libqglviewer](https://github.com/microsoft/vcpkg/tree/master/ports/libqglviewer) | |
