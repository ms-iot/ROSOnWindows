# Adding a new port to VCPKG for use in a ROS Node
vcpkg has thousands of recipies for many different cross platform libraries. If you find that a dependency is not part of vcpkg, this walkthrough will help you get started. Every package has its own idiosyncrasies, so this is only a general ROS specific guide. Please refer to the documentation 


## Porting a C++ Dependency with vcpkg
Before you begin, you'll want to fork vcpkg into your github account. Next, Fix up your the ROS environment to use your fork.

```
cd c:\opt\vcpkg
git remote add upstream https://github.com/microsoft/vcpkg
git remote set-url origin https://github.com/<your github>/vcpkg
git fetch --all
.\bootstrap-vcpkg.bat
```

Now we will add a port fpr the package. We will follow the process outlined on [Packaging a Github Repo](https://github.com/microsoft/vcpkg/blob/master/docs/examples/packaging-github-repos.md) &nearr;. 

Create a folder for the package you'd like to port, then add `CONTROL` and `portfile.cmake` files:

*CONTROL*

```
Source: <name>
Version: <latest release version>
Homepage: https://github.com/<organization>/<project>
Description: <Description from github>
```

*portfile.cmake*

```
include(vcpkg_common_functions)

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO <organization>/<project>
    REF <latest release version>
    SHA512 1
    HEAD_REF master
)

vcpkg_configure_cmake(
    SOURCE_PATH ${SOURCE_PATH}
    PREFER_NINJA
)
vcpkg_install_cmake()
file(INSTALL ${SOURCE_PATH}/COPYING DESTINATION ${CURRENT_PACKAGES_DIR}/share/<project> RENAME copyright)
```

Then build with `vcpkg build <project>:x64-windows`

Which will error because of the SHA512 hash mismatch:

```
  File does not have expected hash:

          File path: [ C:/opt/vcpkg/downloads/temp/<package> ]
      Expected hash: [ 1 ]
        Actual hash: [ 02a61de205bd1dd116677cf4c530d7adb689442252aeafdc549d54531a62ab10e999062403ddb8aed3d89e4f248ad10c0998739b33004ec02e9914150854d47c ]
```

Which is replaced in the *portfile* above:

```
    SHA512 02a61de205bd1dd116677cf4c530d7adb689442252aeafdc549d54531a62ab10e999062403ddb8aed3d89e4f248ad10c0998739b33004ec02e9914150854d47c
```
Then build again with `vcpkg build <package>:x64-windows`

You may see errors message during the build. If so, you'll need to fix package and optionally provide a vcpkg patch file.

## Patching during vcpkg build

We now need to change some code in order to make mavlink build correctly on Windows. Following [vcpkg patching documentation](https://vcpkg.readthedocs.io/en/latest/examples/patching/), we will do the following.

Seed the patch by adding the original unpacked sources (this won't be checked in):

```
cd c:\opt\vcpkg\buildtrees\<package>
git init
git add .
git commit -m "create patch"
```

Next, fix the sources and attempt to build. Once builds complete and testing succeeds, you can create a vcpkg patch:


```
cd c:\opt\vcpkg\buildtrees\<package>\....
git diff > ..\..\..\..\ports\<port>\<path name>.patch
```

And include it in the portfile:

```
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO <organization>/<project>
    REF <latest release version>
    SHA512 02a61de205bd1dd116677cf4c530d7adb689442252aeafdc549d54531a62ab10e999062403ddb8aed3d89e4f248ad10c0998739b33004ec02e9914150854d47c
    HEAD_REF master
    PATCHES
      "generate-msgs.patch"
)
```

## Committing a vcpkg
Once you've successfully built and tested the vcpkg, you can submit the code and issue a pull request to the vcpkg repository.

