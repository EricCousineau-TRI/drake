# Temp benchmark

To run:

```sh
git fetch https://github.com/EricCousineau-TRI/drake feature-pr13752-benchmark-wip
git checkout FETCH_HEAD
# Avoid "tearing" in script: https://stackoverflow.com/a/63234198/7829525
cat ./tmp/run_benchmark.sh | bash
```
