# To setup:

```sh
cd drake
cd tmp
git clone --recursive https://github.com/RussTedrake/manipulation manipulation_repo
find manipulation_repo -name 'BUILD.bazel" | xargs rm

cd ..
bazel build //tmp:repro
bazel-bin/tmp/repro
```
