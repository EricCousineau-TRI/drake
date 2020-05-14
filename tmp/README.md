For: <https://github.com/RobotLocomotion/drake/issues/13317>

# Repro on Ubuntu

```sh
cd drake

cd tmp
python3 -m virtualenv -p python3 --system-site-packages ./venv
./venv/bin/pip install -I numpy==1.18.4

bazel build //bindings/pydrake:py/autodiffutils_test
./venv/bin/python ../bazel-bin/bindings/pydrake/py/autodiffutils_test
```
