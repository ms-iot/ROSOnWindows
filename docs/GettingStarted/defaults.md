```yaml
{
    "build": {
        "symlink-install": true,
        "event-handlers": ["console_cohesion+"],
        "catkin-cmake-args": [
            "-DCATKIN_SKIP_TESTING=ON",
            "-DCMAKE_PREFIX_PATH=c:/miniconda/envs/myenv/library"
        ],
        "install-base": "c:\\miniconda\\envs\\myenv\\library",
        "parallel-workers" : 1,
        "merge-install": true
    },
    "test": {
        "merge-install": true,
        "install-base": "c:\\miniconda\\envs\\myenv\\library"
    }
}
```