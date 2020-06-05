---
title: ROS 2 Binary Installation
---

# ROS 2 on Windows Setup

## Windows Operating System

* ROS for Windows requires 64-bit Windows 10 Desktop or Windows 10 IoT Enterprise.
* Please ensure that you have Powershell installed and in the system path.
* Exclude c:\opt (and later your workspace folder) from real-time virus Scanners, as they can interfere with install and development.

## Reserve space for the installation

* Clean and back up any existing data under c:\opt before proceeding.
* c:\opt is the required install location. Relocation is not currently enabled.
* Please ensure you have 10 GB of free space on the C:\ drive for the installation and development.

## Install Visual Studio 2019

Building a ROS project for Windows requires Visual Studio and the Microsoft SDKs for Windows.

* [Download Visual Studio 2019](https://docs.microsoft.com/en-us/cpp/build/vscpp-step-0-installation?view=vs-2019)
    * Vcpkg is used for managing dependent libraries. It requires that the English language pack be installed.
    * Include "Desktop development with C++" workload.
    * If you already have Visual Studio 2019 installed, you can [Modify Installation](https://docs.microsoft.com/en-us/visualstudio/install/modify-visual-studio?view=vs-2019)

## Install Windows Package Manager

Chocolatey is a package manager for Windows. It is used to make it easy to install tools and libraries needed for building and running ROS projects. The following instructions redirect the chocolatey install location into the c:\opt, so that you can clean or move a ROS environment from that one location.

1. In the Start Menu, find the "x64 Native Tools Command Prompt for VS 2019" item.
2. Right Click, select More then "Run as Administrator"
3. Copy the following command line:

    ```
    @"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -InputFormat None -ExecutionPolicy Bypass -Command "iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'))" && SET "PATH=%PATH%;%ALLUSERSPROFILE%\chocolatey\bin"
    ```

    * Paste it into the command window.
    * Approve any prompts
    * Once it has completed, close the command prompt to complete the install.

4. Install Git:

    * Reopen the Visual Studio Command Window as described above.
    * Please install Git using the command here, even if you have it installed as an application:

        ```
        choco upgrade git -y
        ```

    * Close and Reopen the Visual Studio Command Window as described above.
    * Ensure Git is now available in the Visual Studio command window:

        ```
        git --version
        ```

## Installing ROS 2 Binaries

1. From the start menu, look for [`x64 Native Tools Command Prompt for VS 2019`][vsdevcmd].
2. Open the command prompt as administrator.
3. Run the following to install `ROS 2 Eloquent` and `Gazebo 9`.

```Batchfile
mkdir c:\opt\chocolatey
set ChocolateyInstall=c:\opt\chocolatey
choco source add -n=ros-win -s="https://roswin.azurewebsites.net/api/v2" --priority=1
choco upgrade ros-eloquent-desktop gazebo9 wget -y --execution-timeout=0 --pre
```

You can close the command prompt now.

Now you have ROS 2 `ros-eloquent-desktop` and `gazebo9` nightly build installed. 

## Create bootstrap script

Open your favorite editor and create a file `bootstrap.bat` under `c:\opt\ros\eloquent\x64\bootstrap.bat` with the following content:

```Batchfile
set "ChocolateyInstall=c:\opt\chocolatey"
set "PATH=c:\opt\chocolatey\bin;C:\opt\python37amd64\DLLs;C:\opt\python37amd64\;C:\opt\python37amd64\Scripts;c:\opt\vcpkg\installed\x64-windows\bin;c:\opt\vcpkg;c:\opt\rosdeps\x64\bin;C:\opt\rosdeps\x64\lib;C:\opt\rosdeps\x64\tools\protobuf;%PATH%"
call "C:\opt\ros\eloquent\x64\local_setup.bat"
call "C:\opt\rosdeps\x64\share\gazebo-9\setup.bat"
set "CMAKE_PREFIX_PATH=c:\opt\vcpkg\installed\x64-windows;c:\opt\rosdeps\x64;%CMAKE_PREFIX_PATH%"
set "ROS_PACKAGE_PATH=C:\opt\ros\eloquent\x64\share"
set "ROS_ETC_DIR=C:\opt\ros\eloquent\x64\etc\ros"
```

## Open a Developer Command Prompt

1. From the start menu, look for [`x64 Native Tools Command Prompt for VS 2019`][vsdevcmd].
2. Run the shortcut as administrator.
3. Once the developer command prompt is open, run

```
c:\opt\ros\eloquent\x64\bootstrap.bat
```

Now you are in the ROS 2 Developer command prompt.

