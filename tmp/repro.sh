#!/bin/bash
set -eux -o pipefail

bash-isolate() {
     "$@";
}
# Ensure we have no external environment.
if [[ -z ${_isolate:+D} ]]; then
    exec env -i \
        _isolate=1 \
        HOME=$HOME DISPLAY=$DISPLAY SHELL=$SHELL TERM=$TERM USER=$USER PATH=/usr/local/bin:/usr/bin:/bin \
        bash --norc $0 "$@"
fi

cd $(dirname $0)

if [[ ! -d ./venv ]]; then
    python3 -m virtualenv -p python3 ./venv --system-site-packages
    ./venv/bin/pip install torch==1.0.0
fi

python=${PWD}/venv/bin/python

cat > ../tools/py_toolchain/interpreter_paths.bzl <<EOF
LINUX_INTERPRETER_PATH = "${python}"
LINUX_DBG_INTERPRETER_PATH = "borked"
MACOS_INTERPRETER_PATH = "borked"
EOF

bazel build //tmp:repro_issue12073
bin=../bazel-bin/tmp/repro_issue12073
export PYTHONUNBUFFERED=1

run() {
    label=$1
    shift
    { strace -o /tmp/strace.txt "$@" || :; } 2>&1 | tee /tmp/output.txt
    ./filt.py /tmp/output.txt ./output_${label}
    ./filt.py --for_strace /tmp/strace.txt ./strace_${label}
}

run last ${bin}  # This should work
run first ${bin} --torch_first
