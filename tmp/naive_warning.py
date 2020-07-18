# Copied from `pydrake/__init__.py` for isolated repro.
import sys

import torch


def _check_for_rtld_global_usages():
    # Naively check if `torch` is using RTLD_GLOBAL. For more information, see
    # the above _RTLD_GLOBAL_WARNING message, #12073, and #13707.
    torch = sys.modules.get("torch")
    if torch is None:
        return False
    # This symbol was introduced in v1.5.0 (pytorch@ddff4efa2).
    # N.B. Per investigation in #13707, it seems like torch==1.4.0 also plays
    # better with pydrake. However, we will keep our warning conservative.
    using_rtld_global = getattr(torch, "USE_RTLD_GLOBAL_WITH_LIBTORCH")
    if not using_rtld_global:
        return False
    init_file = getattr(torch, "__file__")
    if init_file.endswith(".pyc"):
        init_file = init_file[:-1]
    if not init_file.endswith(".py"):
        return False
    with open(init_file) as f:
        init_source = f.read()
    return "sys.setdlopenflags(_dl_flags.RTLD_GLOBAL" in init_source


print(f"torch=={torch.__version__}")
if _check_for_rtld_global_usages():
    print("WARNING!!!")
else:
    print("Good: No warning")
