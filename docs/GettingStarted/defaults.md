```yaml
{
    "build": {
        "symlink-install": true,
        "cmake-args": [
            "-G",
            "Ninja",
            "-DCATKIN_SKIP_TESTING=ON",
            "-DCMAKE_BUILD_TYPE=Release"],
        "event-handlers": ["console_cohesion+"],
        "parallel-workers" : 1,
        "merge-install": true
    }
}
```