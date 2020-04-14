```yaml
name: <ros package> CI
on:
  pull_request:
  push:
    branches:
      - master
  schedule:
    # Run every hour. This helps detect flakiness,
    # and broken external dependencies.
    - cron:  '0 * * * *'

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
        package-name: <ros package>
        vcs-repo-file-url: ${{ github.workspace }}/ci/deps.rosinstall
        extra-cmake-args: "-G Ninja -DCMAKE_TOOLCHAIN_FILE=c:/ci/toolchain.cmake -DCATKIN_SKIP_TESTING=ON -DCMAKE_BUILD_TYPE=Release"
      env:
        COLCON_DEFAULTS_FILE: ${{ github.workspace }}/ci/defaults.yaml
        ROS_PYTHON_VERSION: 3
        CC: cl.exe
        CXX: cl.exe
```
