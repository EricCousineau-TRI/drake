from __future__ import absolute_import, print_function

import ctypes
import numpy as np

"""
@file
Defines a mapping between Python and alias types, and provides canonical Python
types as they relate to C++.
"""

def _get_type_name(t):
    # Gets type name as a string.
    if t.__module__ != "__builtin__":
        return t.__module__ + "." + t.__name__
    else:
        return t.__name__


class _StrictMap(object):
    # Provides a map which may only add a key once.
    def __init__(self):
        self._values = dict()

    def add(self, key, value):
        assert key not in self._values, "Already added: {}".format(key)
        self._values[key] = value

    def get(self, key, default):
        return self._values.get(key, default)


class _AliasRegistry(object):
    # Registers aliases for a set of objects. This will be used for template
    # parameters.
    def __init__(self):
        self._to_canonical = _StrictMap()

    def register(self, canonical, *aliases):
        for alias in aliases:
            self._to_canonical.add(alias, canonical)

    def get_canonical(self, alias, default_same=True):
        # Get registered canonical type if there is a mapping; otherwise return
        # original type.
        default = alias
        if not default_same:
            default = None
        return self._to_canonical.get(alias, default)

    def get_name(self, alias):
        canonical = self.get_canonical(alias)
        if isinstance(canonical, type):
            return _get_type_name(canonical)
        else:
            # For literals.
            return str(canonical)


# Create singleton instance.
_aliases = _AliasRegistry()

# Register common Python aliases relevant for C++.
_register = _aliases.register
_register(float, ctypes.c_double, np.double)
_register(np.float32, ctypes.c_float)
_register(int, np.int32, ctypes.c_int32)
_register(np.uint32, ctypes.c_uint32)


def get_types_canonical(param):
    """Gets the canonical types for a set of Python types (canonical as in
    how they relate to C++ types. """
    return tuple(map(_aliases.get_canonical, param))


def get_type_names(param):
    """Gets the canonical type names for a set of Python types (canonical as in
    how they relate to C++ types. """
    return tuple(map(_aliases.get_name, param))
