# -*- python -*-

# Default value of interpreter_path used by the py_runtime in the default
# Python toolchain registered on the @platforms//os:linux platform.
# LINUX_INTERPRETER_PATH = "/usr/bin/python3"
LINUX_INTERPRETER_PATH = "/home/eacousineau/.local/opt/cpython/3.8.2/bin/python3.8"

# Default value of interpreter_path used by the py_runtime in the Python debug
# toolchain registered on the @platforms//os:linux platform when the
# --extra_toolchains=//tools/py_toolchain:linux_dbg_toolchain command line
# option is given.
LINUX_DBG_INTERPRETER_PATH = "/usr/bin/python3-dbg"

# Default value of interpreter_path used by the py_runtime in the default
# Python toolchain registered on the @platforms//os:osx platform.
MACOS_INTERPRETER_PATH = "/usr/local/opt/python@3.8/bin/python3"
