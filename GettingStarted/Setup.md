![ROS Logo](http://www.ros.org/wp-content/uploads/2013/10/rosorg-logo1.png)

> ROS on Windows is experimental.

# Setup

## Prerequisites
+ Operating System
    + ROS1 for Windows requires Windows 10 Desktop or Windows 10 IoT Enterprise.
    + ROS1 for Windows requires 64-bit Windows installation.
    + ROS1 is not currently enabled on Windows 10 IoT Core.

+ Reserve space for the installation
    + Clean and back up any existing data under `c:\opt` before proceeding. `c:\opt` will be the root location for the later installation.
    + Reserve 10 GB free space under C drive for the installation and later development.

+ Install Visual Studio 2017 Community, Professional or Enterprise
    + Building a ROS project for Windows requires Visual Studio and the Microsoft SDKs for Windows.
    + [Download Visual Studio 2017 Community or Professional Edition](https://visualstudio.microsoft.com/) 
        + Include C++ Development
+ Create an Administrative command line shortcut for Visual Studio:
    + To Create an shortcut
        + Right click in a Windows Explorer folder, select New > Shortcut
    + In the shortcut path, copy the highlighted command line from the following options, depending on the Visual Studio install above:
        + If you are using Community:
            `C:\Windows\System32\cmd.exe /k "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64`
        + If you are using Professional:
            `C:\Windows\System32\cmd.exe /k "C:\Program Files (x86)\Microsoft Visual Studio\2017\Professional\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64`
        + If you are using Enterprise:
            `C:\Windows\System32\cmd.exe /k "C:\Program Files (x86)\Microsoft Visual Studio\2017\Enterprise\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64`
    + Name the shortcut *ROS*
    + Set that shortcut as Administrator
        + Right Click on the shortcut.
        + Select the Shortcut Tab if not already selected.
        + Press the Advanced button
        + Check the button "Run as Administrator".
        + Press OK on the Advanced properties dialog.
        + Press OK on the "ROS Properties" shortcut dialog.
 
+ Install [Chocolatey](https://chocolatey.org/)
    + Chocolatey is a package manager for Windows. It is used to make it easy to install tools and libraries needed for building and running ROS projects.
    + Open the ROS Command Prompt created above and approve the administrative elevation if not already opened.
    + Copy the following command line: 
        `@"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -InputFormat None -ExecutionPolicy Bypass -Command "iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'))" && SET "PATH=%PATH%;%ALLUSERSPROFILE%\chocolatey\bin" `
    + Paste it into the command Window.
    + Approve any prompts
    + Once it has completed, close the command prompt to complete the install.

+ Install Git:
    +  Open the ROS Command Prompt created above and approve the administrative elevation if not already opened.
    + `choco install git -y`

## Binary Package Installation
To set up ROS for Windows follow these recommended steps:

### Install ROS for Windows
To get things started, install the recommended `desktop` metapackage. A Metapackage is a collection of other packages. The Desktop metapackage refers to a number of other packages needed to build, run, debug and visualize a robot.

+  Open the ROS Command Prompt created above and approve the administrative elevation if not already opened.
    + `choco source add -n=ros-win -s="https://roswin.azurewebsites.net/api/v2" --priority=1`
    + `choco upgrade ros-melodic-desktop -y`

Congratulations! You have successfully installed ROS for Windows after all the previous steps. Now you are all set to further explore [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) with ROS for Windows.

### Stay Up to Date
Builds are updated nightly. If you want to move your tree forward, use Chocolatey's upgrade feature:

+  Open the ROS Command Prompt created above and approve the administrative elevation if not already opened.
    + Run the following command:
    + `choco upgrade ros-melodic-desktop -y`
    + Close the command window.
