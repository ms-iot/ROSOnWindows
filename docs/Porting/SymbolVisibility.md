# Symbol Visibility in ROS on Windows
## Background
The Microsoft Visual C++ Compiler (MSVC) exposes symbols from a Dynamic Link Library (DLL) only if they are explicitly exported. The clang and gcc compilers have a have an option to do the same, but it is off by default. As a result, when a library previously built on Linux is built on Windows, other libraries may be unable to resolve the external symbols.

Below are examples of common error messages which can be caused by symbols not being exposed:

```bat
error C2448: '__attribute__': function-style initializer appears to be a function definition
'visibility': identifier not found
```
```bat
CMake Error at C:/ws_ros2/install/random_numbers/share/random_numbers/cmake/ament_cmake_export_libraries-extras.cmake:48 (message):
  Package 'random_numbers' exports the library 'random_numbers' which
  couldn't be found
```

The two main solutions to this are adding visibility control headers and the WINDOWS_EXPORT_ALL_SYMBOLS property. It is recommended to use visibility control headers as they are more deterministic and provide other benefits such as binary size and link times. These benefits apply when building with GCC as well.

## Visibility Control Headers

Open Robotics recommends using a deterministic method to ensure symbol visibility. This method is to use visibility control headers. The purpose of these headers is to define a macro for each library which correctly declares symbols as dllimport or dllexport. This is decided based on whether the library is being consumed or being built itself. The logic in the macro also takes the compiler into account and includes logic to select the appropriate syntax.

The following link includes step by step instructions for adding explicit symbol visibility to a library “yielding the highest quality code with the greatest reductions in binary size, load times and link times”: [The GCC visibility documentation](https://gcc.gnu.org/wiki/Visibility).

A `visibility_control.hpp` header like the one in the example below can be placed in the `includes` folder for each library with a unique macro name.

### Example: Robot State Visibility Control Header

This example shows how a visibility control header was added for the `robot_state` library within `movit_core`.
Add a visibility header to the include folder for the library. The boiler plate logic is used with the library name used in the macro to make it unique in the project. In another library, `ROBOT_STATE` would be replaced with the library name.
```c++ 
// Copyright (c) 2019, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Open Source Robotics Foundation, Inc. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef ROBOT_STATE__VISIBILITY_CONTROL_HPP_
#define ROBOT_STATE__VISIBILITY_CONTROL_HPP_
// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOT_STATE_EXPORT __attribute__ ((dllexport))
    #define ROBOT_STATE_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOT_STATE_EXPORT __declspec(dllexport)
    #define ROBOT_STATE_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOT_STATE_BUILDING_LIBRARY
    #define ROBOT_STATE_PUBLIC ROBOT_STATE_EXPORT
  #else
    #define ROBOT_STATE_PUBLIC ROBOT_STATE_IMPORT
  #endif
  #define ROBOT_STATE_PUBLIC_TYPE ROBOT_STATE_PUBLIC
  #define ROBOT_STATE_LOCAL
#else
  #define ROBOT_STATE_EXPORT __attribute__ ((visibility("default")))
  #define ROBOT_STATE_IMPORT
  #if __GNUC__ >= 4
    #define ROBOT_STATE_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOT_STATE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOT_STATE_PUBLIC
    #define ROBOT_STATE_LOCAL
  #endif
  #define ROBOT_STATE_PUBLIC_TYPE
#endif
#endif  // ROBOT_STATE__VISIBILITY_CONTROL_HPP_
```
To use the macro, add `ROBOT_STATE_PUBLIC` before symbols which need to be visible to external libraries. For example:
 
```c++
Class ROBOT_STATE_PUBLIC example_class {}
```

```c++
ROBOT_STATE_PUBLIC void example_function (){}
``` 

## WINDOWS_EXPORT_ALL_SYMBOLS Target Property
CMake implements a macro which will export all symbols on Windows. The `WINDOWS_EXPORT_ALL_SYMBOLS` property turns all `add_library` calls which do not specify type to shared builds. Calls to `add_library` build the specified library from source as either `STATIC`, `SHARED` or `MODULE`. Without the `WINDOWS_EXPORT_ALL_SYMBOLS` property, if no type is specified it will default to either `STATIC`, `SHARED` based on the value of the `BUILD_SHARED_LIBS` variable.

The property can be implemented by adding the following to the CMakeLists file:
`set_target_properties(${LIB_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)`

Note: if there is more than one library in a CMakeLists file you will need to call `set_target_properties` on each of them separately.

There is an exception to this method in the case of global data such as static data members in classes. For example:

```c++
class KinematicsBase
{
public:
  static const rclcpp::Logger LOGGER;
…
```

In these cases dllimprort/dllexport must be applied explicitly. This can be done using generate_export_header as described in the following article:
[Create dlls on Windows without declspec() using new CMake export all feature](https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/).
