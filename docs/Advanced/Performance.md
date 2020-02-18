# Performance
This page covers guidelines for optimizing on Robot performance on Windows.

## Windows 10 IoT Enterprise LTSC
Windows 10 IoT Enterprise is version of Windows intended for embedded applications. *IoT* isn't just about Lightbulbs and Thermostats (Windows 10 IoT Core is good for that), it's about network connected devices - including Robots. Windows 10 IoT Enterprise is a full desktop version of Windows, with a license to use on dedicated purpose devices. 

Windows 10 IoT Enterprise offers a Long Term Support Channel (also called LTSC), which allows you to lock a version with 10 years of support for a fixed up front cost. The LTSC version of Windows 10 IoT Enterprise offers an overall smaller on-disk and runtime footprint.

You can [try an evaluation of Windows 10 IoT Enterprise](https://www.microsoft.com/en-us/evalcenter/evaluate-windows-10-enterprise), then talk to a [Windows 10 IoT Enterprise distributor to commercialize](https://go.microsoft.com/fwlink/?linkid=2094697).


## Fair Process Scheduling
By default, Windows prioritizes foreground applications. This can cause a 'unfair' scheduling of ROS nodes, which can impact throughput in a ROS composition. To make scheduling more fair, consider setting the priority control flag to prioritize background applications (`0x18`):

```no-highlight
reg add HKLM\SYSTEM\CurrentControlSet\Control\PriorityControl /v Win32PrioritySeparation /t REG_DWORD /d 0x18 /f
```

## Per process priority
Adjusting the process scheduler can dramatically improve the flow of messages through a ROS composition. However, some ROS nodes might require a higher scheduling priority than others. To address this, there are two models you can use - If you have source access, you can tune the process scheduling using the `SetPriorityClass` API; otherwise you can use command line tool per process.

### Priority Class API
Please refer to the documentation on [SetPriorityClass](https://docs.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-setpriorityclass). This API allows you to adjust the scheduling priority of a process from the ROS sources. 

> NOTE: While tempting, it is best to avoid the `REALTIME_PRIORITY_CLASS` as this setting can be detemental to overall system performance.

### Command Line
After you start a rosgraph with `roslaunch`, you can modify the priority of specific nodes using a PowerShell script.

``` powershell
(get-process ros_winml_node).PriorityClass='AboveNormal'
```

### Ingegrating cross platform prioritization directly into ROSLaunch files.
Wouldn't that be cool?

## Disabling Services
Using Windows 10 Desktop or Windows 10 IoT Enterprise comes with many services which are useful for general computing and enterprise scenarios. 

For Robotics, many services are not required. All services consume resources, so disabling unnessesary services would improve performance. During deployment, you should evaluate shich services are required by your scenario on a case by case basis. 

Please refer to this documentation for [disabling Windows Services](https://docs.microsoft.com/en-us/windows-server/security/windows-services/security-guidelines-for-disabling-system-services-in-windows-server) (guidance is applicable to Windows 10 IoT).


## High Resolution Time
Windows Provides APIs for time using `QueryPerformanceFrequency` and `QueryPerformanceCounter` which returns the raw timer values. ROS time uses these APIs internally; however, you can use these high resolution timing outside of the ROS APIs.

for more information, please refer to the [High Resolution timestamp documentation](https://docs.microsoft.com/en-us/windows/win32/sysinfo/acquiring-high-resolution-time-stamps).

```C++
LARGE_INTEGER StartingTime, EndingTime, ElapsedMicroseconds;
LARGE_INTEGER Frequency;

QueryPerformanceFrequency(&Frequency); 
QueryPerformanceCounter(&StartingTime);

// Activity to be timed

QueryPerformanceCounter(&EndingTime);
ElapsedMicroseconds.QuadPart = EndingTime.QuadPart - StartingTime.QuadPart;


//
// We now have the elapsed number of ticks, along with the
// number of ticks-per-second. We use these values
// to convert to the number of elapsed microseconds.
// To guard against loss-of-precision, we convert
// to microseconds *before* dividing by ticks-per-second.
//

ElapsedMicroseconds.QuadPart *= 1000000;
ElapsedMicroseconds.QuadPart /= Frequency.QuadPart;
```

