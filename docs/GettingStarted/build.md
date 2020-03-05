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
    - uses: ros-tooling/action-ros-ci@master
      with:
        package-name: winml_tracker
    - uses: actions/upload-artifact@master
      with:
        name: colcon-logs
        path: ros_ws/log
```
