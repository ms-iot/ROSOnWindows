```yaml
{
    "build": {
        "symlink-install": true,
        "event-handlers": ["console_cohesion+"],
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