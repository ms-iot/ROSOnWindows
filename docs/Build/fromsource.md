
## Install ROS for Windows from source
As an alternative to using Chocolatey, ROS for Windows can also be [installed from source](source.md).

## ROS Environment Command Prompt
When running ROS, open an elevated Command Prompt with the following setup:
```bat
c:\opt\ros\melodic\x64\setup.bat
```

If you are building catkin projects, use the Visual Studio x64 command line shortcut that was created earlier to launch a Command Prompt or execute the following command in the current Command Prompt to [make Visual Studio build tools discoverable](https://docs.microsoft.com/en-us/dotnet/csharp/language-reference/compiler-options/how-to-set-environment-variables-for-the-visual-studio-command-line) for CMake:
```bat
"c:\Program Files (x86)\Microsoft Visual Studio\2017\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64
```
