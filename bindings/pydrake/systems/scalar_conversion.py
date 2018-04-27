"""Provides simple utilities to aid in scalar type conversion."""

import copy

from pydrake.systems.framework import SystemScalarConverter
from pydrake.util.cpp_template import is_instantiation_of


def _to_scalar(item):
    assert len(item) == 1
    return item[0]


_converter_cache = {}


def _get_conversion_pairs(param_list):
    # Intersect.
    T_compat = []
    for T, in param_list:
        if T in SystemScalarConverter.SupportedScalars:
            T_compat.append(T)
    # Outer join without duplicates.
    param_pairs = []
    for T in T_compat:
        for U in T_compat:
            if T != U:
                param_pairs.append((T, U))
    return param_pairs


def _make_converter(template, param_pairs=None, use_cache=True):
    cache_key = (template, param_pairs)
    if use_cache:
        cache_entry = _converter_cache.get(cache_key)
        if cache_entry:
            return copy.copy(cache_entry)

    if param_pairs is None:
        param_pairs = _get_conversion_pairs(template.param_list)

    # Generate and register each conversion.
    converter = SystemScalarConverter()
    # Define capture to ensure the current values are bound, and do not change
    # through iteration.
    def add_captured(param):
        T, U = param

        def conversion(system):
            assert isinstance(system, template[U])
            return template[T](system)

        converter.Add[T, U](conversion)

    map(add_captured, param_pairs)

    if use_cache:
        _converter_cache[cache_key] = copy.copy(converter)
    return converter


class ScalarHelper(object):
    """A helper to handle dispatching constructors and help creating
    converters."""
    def __init__(self, template, ):
        self._template = template

    def make_converter(self, param_pairs=None, use_cache=True):
        """Creates system scalar converter for a given template, assuming that
        the System class has a scalar-type-converting copy constructing.
        @see check_if_copy_constructor

        @param template TemplateClass instance.
        @param param_pairs List of pairs, (T, U), defining a conversion from a
            scalar type of U to T.
            If None, this will use all possible pairs that the Python bindings
            of `SystemScalarConverter` support.
        @param use_cache Use cache from a global store, returning a copy on a
            cache hit.
        """
        return _make_converter(self._template, param_pairs, use_cache)

    def check_if_copying(self, args, kwargs):
        """Determines if constructor call has one parameter which is an
        instantiation of the current template, indicating that this is a System
        scalar type conversion copy constructor.

        @param args Positional arguments to function.
        @param kwargs Keyword arguments to constructor.
        @return The first argument (other system) if it is, None otherwise.
        """
        if len(args) == 1 and len(kwargs) == 0:
            if is_instantiation_of(type(args[0]), self._template):
                return args[0]
        return None
