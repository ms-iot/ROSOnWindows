```yaml
name: Build
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
    - uses: ros-tooling/setup-ros@master
    - uses: ros-tooling/action-ros-ci@master
      with:
        package-name: <your package name>
        vcs-repo-file-url: ${{ github.workspace }}/ci/deps.rosinstall
      env:
        COLCON_DEFAULTS_FILE: ${{ github.workspace }}/ci/defaults.yaml
        ROS_PYTHON_VERSION: 3
        CC: cl.exe
        CXX: cl.exe
```