# Using a C++ Dependency in a ROS Node using vcpkg
If you find that a dependency is not met with vcpkg today, that dependency can be added by following the [vcpkg guide](AddingVCPKG.md)

### Updating vcpkg mappings
Often the name of a a library on Linux differs from the name on vcpkg. In order to address this, a mapping file called `vcpkg.yaml` is part of the rosditro database. This creates a mapping between the Linux name for the package and the vcpkg name.

* Fork [https://github.com/ms-iot/rosdistro-db](https://github.com/ms-iot/rosdistro-db) &nearr; into your github account, if you haven't already
* Create a file called `0-update.list` in `c:\opt\ros\melodic\x64\etc\ros\rosdep\sources.list.d`
* In this file, add a line which points to your fork.
```
# os-specific listings first
yaml https://raw.githubusercontent.com/<your github>/rosdistro-db/init_windows/rosdep/win-chocolatey.yaml windows
yaml https://raw.githubusercontent.com/<your github>/rosdistro-db/init_windows/rosdep/vcpkg.yaml windows
```
* Add a mapping from the dependency name used in the ROS package, to the name used in vcpkg. You can edit this file directly on github in your fork. 
  The format of the vcpkg.yaml:

`Python`
```
<python-package-name>:
    windows:
      pip:
        packages: [<python-package-name-in-pip>]
```
  `C++`
```
  <linux--package-name>:
    windows:
      vcpkg:
        packages: [<vcpkg-name>]
```

* Update rosdeps on your computer.
```
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
You'll see that the dependency resolves correctly. However, you may be provided with specific instructions for using that library in ROS nodes:
```
The package <library>:x64-windows provides CMake targets:

    find_package(<package> CONFIG REQUIRED)
    target_link_libraries(main PRIVATE <package> <package>::<namespace>)
```




























