# Using a C++ Dependency in a ROS Node using vcpkg
If you find that a dependency is not met with vcpkg today, that dependency can be added by following the [vcpkg guide](AddingVCPKG.md)

### Updating vcpkg mappings
Often the name of a a library on Linux differs from the name on vcpkg. In order to address this, a mapping file called `vcpkg.yaml` is part of the rosditro database. This creates a mapping between the Linux name for the package and the vcpkg name.

* Fork [https://github.com/ms-iot/rosdistro-db](https://github.com/ms-iot/rosdistro-db) &nearr; into your github account, if you haven't already
* Create a file called `0-update.list` in `c:\opt\ros\melodic\x64\etc\ros\rosdep\sources.list.d`
* In this file, add a line which points to your fork.
```no-highlight
# os-specific listings first
yaml https://raw.githubusercontent.com/<your github>/rosdistro-db/init_windows/rosdep/win-chocolatey.yaml windows
yaml https://raw.githubusercontent.com/<your github>/rosdistro-db/init_windows/rosdep/vcpkg.yaml windows
```
* Add a mapping from the dependency name used in the ROS package, to the name used in vcpkg. You can edit this file directly on github in your fork. 
  The format of the vcpkg.yaml:

`Python`
```no-highlight
<python-package-name>:
    windows:
      pip:
        packages: [<python-package-name-in-pip>]
```
  `C++`
```no-highlight
  <linux--package-name>:
    windows:
      vcpkg:
        packages: [<vcpkg-name>]
```

* Update rosdeps on your computer.
```no-highlight
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
You'll see that the dependency resolves correctly. However, you may be provided with instructions for using that library in ROS nodes:
```no-highlight
The package <library>:x64-windows provides CMake targets:

    find_package(<package> CONFIG REQUIRED)
    target_link_libraries(main PRIVATE <package> <package>::<namespace>)
```

These may differ from how linkage happes on Linux. To link this library, to support both Windows and Linux, you can wrap it:

```no-highlight
if (MSVC)
    target_link_libraries(main PRIVATE <package> <package>::<namespace>)
else()
    target_link_libraries(main ... <original>)
endif
```

* Handling RelWithDebinfo
ROS on Windows is delivered with Release binaries that have been built with Debug Info (using cmake's RelWithDebInfo target). When interacting with vcpkg, cmake will map RelWithDebInfo built binaries to Debug binaries. This mismatch will cause problems.

To correct this, add the following to the cmake for the ROS node:
```no-highlight
set_target_properties(${<dependency_LIBRARIES} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)
```




























