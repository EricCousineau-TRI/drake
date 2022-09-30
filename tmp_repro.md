```sh
# cat > ./user.bazelrc <<EOF
# common --action_env=PYTHONPATH=${PWD}/.venv/lib/python3.8/site-packages
# EOF

bazel build //:tmp_repro

# Not needed; can repro on host version.

# python3 -m virtualenv --system-site-packages .venv/
# source .venv/bin/activate

# pip install -U pip wheel
# pip install numpy==1.21.1
# # 1.16 - 1.20 - warning present
# # 1.15 - actually fails, probably due to ABI incompatibility for NumPy build

# deactivate

# bazel run \
#     --test_env=PYTHONPATH=${PWD}/.venv/lib/python3.8/site-packages \
#     //:py/tmp_repro
./bazel-bin/tmp_repro
```
