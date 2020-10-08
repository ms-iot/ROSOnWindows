``` yaml

name: CI
on:
  pull_request:
  push:
    branches:
      - master
  schedule:
    # every Monday
    - cron:  '0 0 * * 1'

jobs:
  build:
    runs-on: [windows-latest]
    strategy:
      fail-fast: false
      matrix:
        ROSDISTRO: [melodic, noetic]
    steps:
    - uses: actions/checkout@v2
      with:
        submodules: recursive
        path: src
    - name: Install
      shell: cmd
      run: |
        choco sources add -n=roswin -s https://aka.ms/ros/public --priority 1
        choco install ros-%ROSDISTRO%-desktop_full -y --no-progress

        : The desktop_full deployment has a chocolatey deployment which is isolated to the ROS distro 
        : (so they don't conflict with each other)
        : If you need other dependencies, they should go after setup.bat in the following block in order
        : to be injected into the isolated deployment.
      env:
        ROSDISTRO: ${{ matrix.ROSDISTRO }}
    - name: Build
      shell: cmd
      run: |
        call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64
        call "C:\opt\ros\%ROSDISTRO%\x64\setup.bat"

        : Additional dependencies after setup.bat.
        : For other ROS repos, remove the : and add the clone commands
        : pushd src
        : git clone https://github.com/ms-iot/audio_common
        : popd

        : For other chocolatey packages, remove the : and add the choco packages
        : choco install <package>

        : For vcpkgs, remove the : and add the vcpkg dependencies.
        : vcpkg install <package>

        catkin_make install -DPYTHON_EXECUTABLE=C:\opt\ros\%ROSDISTRO%\x64\python.exe
      env:
        ROSDISTRO: ${{ matrix.ROSDISTRO }}

    - uses: actions/upload-artifact@v1
      with:
        name: drop
        path: install

```
