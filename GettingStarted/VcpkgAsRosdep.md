<!-- ![ROS Logo](http://www.ros.org/wp-content/uploads/2013/10/rosorg-logo1.png) -->

# Why we marry Vcpkg with Rosdep

Today, many ROS packages consume OSS libraries which are much beyond [ROS on Windows Chocolatey server](https://roswin.azurewebsites.net/) for `Open Robotics` core packages. To enable developers lighting up more ROS packages on Windows, we leverage the Vcpkg community (which has 1000+ OSS libraries ports), and extend the rosdep to manage Vcpkg packages which are known to work with `ROS on Windows` installation. It means ROS developer can either consume the OOS libraries right away if they are in Vcpkg ports or can contribute Vcpkg ports back to the upstream which can benefit ROS developers on Windows community.

# Example Workflow

Let's take [koboki](http://wiki.ros.org/kobuki) as an example. After checking out the source code and running `rosdep check`, two missing requirements are reported from here: `libusb-dev` and `libftdi-dev`. They are the rosdep keys, so you will need another look up to see what's the actual package names for platforms. By checking [`index.ros.org`](https://index.ros.org), they are found as `libusb-dev` and `libftdi-dev` respectively on Ubuntu. By a further check, they are registered as [`libftdi-dev (0.20-4build3)`](https://packages.ubuntu.com/bionic/libftdi-dev) and [`libusb-dev (2:0.1.12-31)`](https://packages.ubuntu.com/bionic/libusb-dev) on [`packages.ubuntu.com`](https://packages.ubuntu.com), which gives us an idea what version to use.

Back to Vcpkg, `libusb-dev` can be found as `libusb-win32`. You can do `vcpkg install libusb-win32:x64-windows` from the ROS command prompt. Since Vcpkg is integrated with your ROS installation already, developers can do `catkin_make` to iterate on Windows port work. And in this example, `libftdi-dev` is not found in Vcpkg. It is because the library is not ported yet in Vcpkg. You can make a Vcpkg port for it, and contributethe Vcpkg recipe back to [Microsoft\Vcpkg](https://github.com/microsoft/vcpkg).

> As of today, a port for  `libftdi-dev` is contributed back to [Vcpkg](https://github.com/microsoft/vcpkg/pull/6843).

After a Windows port finished, developers can send pull request to register Vcpkg port to Rosdep keys (https://github.com/ms-iot/rosdistro-db/blob/init_windows/rosdep/vcpkg.yaml). So the next time for who wants to build koboki, installing the system dependencies can be simplified to by running `rosdep install`.


# How do I try it out

This new integration is avaiable in the build of `20190617.1.0-pre` and above. To try it out, run Chocolatey upgrade to use the prerelease builds:

`choco upgrade ros-melodic-desktop_full -y --pre`


# Feedback

For any feedback, please open an issue on [ms-iot\ROSOnWindows](https://github.com/ms-iot/rosonWindows/issues). We will help you there.
