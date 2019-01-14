# Vcpkg, ROS1 system dependencies, and Azure DevOps CI investigation

In ROS1, there are many open sources project required as a minimum to run ROS desktop stack. In the beginning, we either privately built the libraries by project-specific build instructions or looked for stock binaries if it just works.

This approach works fine for the very initial ROS1 porting to manage the initial set of open sources libraries. However, we are also seeing problems with this method:
* **Effort spent on How-to-build-xyz:** Every new project could come with its owned way to build end-to-end. That requires knowledge to understand a certain level of its owned build details.
* **Updatibility:** The usage of stock binaries, it would be a bottleneck for us to upgrade its downstream projects (thinking about ABI changes or compatiblity), unless we know how to build them.
* **Latest Visual C++ toolchain support:** Not every project also keeps its build instructions or code up-to-dated for the latest Visual C++ toolchain. We want the binaries always come from the best of breed.

To address those issues, we are considering a fantastic project - Vcpkg. Vcpkg manages the how-to-build sauces of eight hundred and more open-source projects, and with framework (Vcpkg helper) support, users are easily to compile libraries cross-platform and using the latest toolchain.
