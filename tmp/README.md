# To setup:

```sh
cd drake
cd tmp
git clone --recursive https://github.com/RussTedrake/manipulation manipulation_repo
find manipulation_repo -name 'BUILD.bazel" | xargs rm
bazel run :repro
```
