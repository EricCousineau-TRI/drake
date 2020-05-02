"""
Actual docstring will be overridden programmatically.
"""


def _import_cc_module_vars(cc_module, py_module_name):
    # Imports the cc_module's public symbols into the named py module
    # (py_module_name), resetting their __module__ to be the py module in the
    # process.
    # Returns a list[str] of the public symbol names imported.
    py_module = sys.modules[py_module_name]
    var_list = []
    for name, value in cc_module.__dict__.items():
        if name.startswith("_"):
            continue
        if getattr(value, "__module__", None) == cc_module.__name__:
            value.__module__ = py_module_name
        setattr(py_module, name, value)
        var_list.append(name)
    return var_list


from . import _module_py  # noqa

__doc__ = _module_py.__doc__
__all__ = _import_cc_module_vars(_module_py, __name__)
