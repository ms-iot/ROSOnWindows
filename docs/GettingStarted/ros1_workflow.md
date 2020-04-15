```yaml

name: CI
on:
  pull_request:
  push:
    branches:
      - master
  schedule:
    # Run it regularly to detect flaky build breaks.
    - cron:  '0 0 */3 * *'

jobs:
  build_and_test:
    runs-on: ${{ matrix.os }}
    strategy:
      fail-fast: false
      matrix:
          os: [windows-latest]
    steps:
    - uses: actions/checkout@v2
    - name: Cleanup windows environment
      shell: bash
      run: |
        rm -rf /c/hostedtoolcache/windows/Boost/1.72.0/lib/cmake/Boost-1.72.0
        mkdir -p /c/ci
        cp $GITHUB_WORKSPACE/ci/toolchain.cmake /c/ci
    - uses: goanpeca/setup-miniconda@v1
      with:
        activate-environment: myenv
        environment-file: ci/environment.yaml
        python-version: 3.7
    - uses: ros-tooling/action-ros-ci@master
      with:
        package-name: <ros pakage>
        vcs-repo-file-url: ${{ github.workspace }}/ci/deps.rosinstall
        extra-cmake-args: "-G Ninja -DCMAKE_TOOLCHAIN_FILE=c:/ci/toolchain.cmake -DCMAKE_BUILD_TYPE=Release"
      env:
        COLCON_DEFAULTS_FILE: ${{ github.workspace }}/ci/defaults.yaml
        ROS_PYTHON_VERSION: 3
        CC: cl.exe
        CXX: cl.exe
    - uses: ros-tooling/action-ros-ci@master
      with:
        package-name: <ros pakage>
        vcs-repo-file-url: ${{ github.workspace }}/ci/empty.rosinstall
        extra-cmake-args: "-G Ninja -DCMAKE_TOOLCHAIN_FILE=c:/ci/toolchain.cmake -DCMAKE_BUILD_TYPE=Release"
      env:
        COLCON_DEFAULTS_FILE: ${{ github.workspace }}/ci/packaging.yaml
        ROS_PYTHON_VERSION: 3
        CC: cl.exe
        CXX: cl.exe
    - uses: actions/upload-artifact@v1
      with:
        name: drop
        path: c:/opt/install

```
