```sh
bazel build //:tmp_repro

python3 -m virtualenv --system-site-packages .venv/
source .venv/bin/activate

pip install -U pip wheel
pip install numpy==1.21.1

./bazel-bin/tmp_repro
```
