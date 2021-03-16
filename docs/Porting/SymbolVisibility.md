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

Two solutions to export symbols on Windows are Visibility Control Headers and the `WINDOWS_EXPORT_ALL_SYMBOLS` property.

Microsoft recommends ROS developers use Visibility Control Headers to control the export of symbols from a binary. Visibility Control Headers provide more control over the symbol export macro and offer other benefits including smaller binary size and reduced link times.

## Visibility Control Headers

The purpose of Visibility Control Headers headers is to define a macro for each library which correctly declares symbols as dllimport or dllexport. This is decided based on whether the library is being consumed or being built itself. The logic in the macro also takes the compiler into account and includes logic to select the appropriate syntax.

The following link includes step by step instructions for adding explicit symbol visibility to a library “yielding the highest quality code with the greatest reductions in binary size, load times and link times”: [The GCC visibility documentation](https://gcc.gnu.org/wiki/Visibility).

A header named `visibility_control.hpp` can be placed in the `includes` folder for each library as shown in the example below.

### Example: Robot State Visibility Control Header

This example shows how a visibility control header would be added for a `my_lib` library with a class called `example_class`.
Add a visibility header to the include folder for the library. The boiler plate logic is used with the library name used in the macro to make it unique in the project. In another library, `MY_LIB` would be replaced with the library name.
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
#ifndef MY_LIB__VISIBILITY_CONTROL_HPP_
#define MY_LIB__VISIBILITY_CONTROL_HPP_
// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MY_LIB_EXPORT __attribute__ ((dllexport))
    #define MY_LIB_IMPORT __attribute__ ((dllimport))
  #else
    #define MY_LIB_EXPORT __declspec(dllexport)
    #define MY_LIB_IMPORT __declspec(dllimport)
  #endif
  #ifdef MY_LIB_BUILDING_LIBRARY
    #define MY_LIB_PUBLIC MY_LIB_EXPORT
  #else
    #define MY_LIB_PUBLIC MY_LIB_IMPORT
  #endif
  #define MY_LIB_PUBLIC_TYPE MY_LIB_PUBLIC
  #define MY_LIB_LOCAL
#else
  #define MY_LIB_EXPORT __attribute__ ((visibility("default")))
  #define MY_LIB_IMPORT
  #if __GNUC__ >= 4
    #define MY_LIB_PUBLIC __attribute__ ((visibility("default")))
    #define MY_LIB_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MY_LIB_PUBLIC
    #define MY_LIB_LOCAL
  #endif
  #define MY_LIB_PUBLIC_TYPE
#endif
#endif  // MY_LIB__VISIBILITY_CONTROL_HPP_
```
To use the macro, add `MY_LIB_PUBLIC` before symbols which need to be visible to external libraries. For example:
 
```c++
Class MY_LIB_PUBLIC example_class {}
```

```c++
MY_LIB_PUBLIC void example_function (){}
``` 

## WINDOWS_EXPORT_ALL_SYMBOLS Target Property
CMake implements a macro which will export all symbols on Windows. The `WINDOWS_EXPORT_ALL_SYMBOLS` property causes function symbols to be automatically exported on windows. More detail of how it works can be found in the [WINDOWS_EXPORT_ALL_SYMBOLS CMake Documentation](https://cmake.org/cmake/help/latest/prop_tgt/WINDOWS_EXPORT_ALL_SYMBOLS.html).

The property can be implemented by adding the following to the CMakeLists file:
`set_target_properties(${LIB_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)`

Note: if there is more than one library in a CMakeLists file you will need to call `set_target_properties` on each of them separately.

There is an exception to this method in the case of global data symbols. For example, a global static data member like the one below.

```c++
class Example_class
{
public:
  static const int Global_data_num;
…
```

In these cases dllimprort/dllexport must be applied explicitly. This can be done using generate_export_header as described in the following article:
[Create dlls on Windows without declspec() using new CMake export all feature](https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/).

## Conclusion

Symbols must be exported on windows as there is no export by default option as with GCC and Clang. Microsoft recomends adding Visibility Control Headers which provide macros to control the export of symbols. The headers provide other benefits to binary size and linker performance, both when building with MSVC or GCC. 