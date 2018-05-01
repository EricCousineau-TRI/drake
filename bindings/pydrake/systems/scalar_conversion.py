"""Provides utilities to aid in scalar type conversion."""

import copy

from pydrake.systems.framework import (
    LeafSystem_,
    SystemScalarConverter,
)
from pydrake.util.cpp_template import (
    _get_module_name_from_stack,
    TemplateClass,
)


def _get_conversion_pairs(T_list):
    # Intersect.
    T_compat = []
    for T in T_list:
        if T in SystemScalarConverter.SupportedScalars:
            T_compat.append(T)
    # Take subset from supported conversions.
    T_pairs = []
    for T, U in SystemScalarConverter.SupportedConversionPairs:
        if T in T_compat and U in T_compat:
            T_pairs.append((T, U))
    return T_pairs


class TemplateSystem(TemplateClass):
    """Provides a means to define templated systems.

    Any class that is added must:
    - Not define `__init__`, as this will be overridden.
    - Define `_construct(self, *args, converter=None)` and
      `_construct_copy(self, *args, converter=None)` instead.
      - `converter` must be present as an argument to ensure that it
        propagates properly.

    If any of these constraints are violated, then an error will be thrown
    at the time of the first class instantiation.

    Example:

        @TemplateSystem.define("MySystem_")
        def MySystem_(T):

            class MySystemInstantiation(LeafSystem_[T]):
                def _construct(self, value, converter=None):
                    LeafSystem_[T].__init__(self, converter)
                    self.value = value

                def _construct_copy(self, other, converter=None):
                    LeafSystem_[T].__init__(self, converter)
                    self.value = other.value

            return MySystemInstantiation

        MySystem = MySystem_[None]  # Default instantiation.

    """
    def __init__(self, name, T_list=None, T_pairs=None, module_name=None):
        """Constructs `TemplateSystem`.

        This differs from `TemplateClass` in that (a) you must pre-specify
        parameters and (b) `.define` is overridden to allow defining `f(T)` for
        convenience.

        @param T_list
            List of T's that the given system supports. By default, it is all
            types supported by `LeafSystem`.
        @param T_pairs List of pairs, (T, U), defining a conversion from a
            scalar type of U to T.
            If None, this will use all possible pairs that the Python bindings
            of `SystemScalarConverter` support.
        @param module_name
            Defining `module_name`, per `TemplateClass`'s constructor.
        """
        if module_name is None:
            module_name = _get_module_name_from_stack()
        TemplateClass.__init__(self, name, module_name=module_name)

        # Stub values to be defined.
        if T_list is None:
            T_list = SystemScalarConverter.SupportedScalars
        for T in T_list:
            assert T in SystemScalarConverter.SupportedScalars, (
                "Type {} is not a supported scalar type".format(T))
        if T_pairs is None:
            T_pairs = _get_conversion_pairs(T_list)
        for T_pair in T_pairs:
            T, U = T_pair
            assert T in T_list and U in T_list, (
                "Conversion {} is not in the original parameter list"
                .format(T_pair))
            assert T_pair in \
                SystemScalarConverter.SupportedConversionPairs, (
                    "Conversion {} is not supported".format(T_pair))

        # Add converter for later access.
        self._T_list = T_list
        self._T_pairs = T_pairs
        self._converter = self._make_converter()

    @classmethod
    def define(cls, name, *args, **kwargs):
        """Provides a decorator which can be used define a scalar-type
        convertible System as a template.

        The decorated function must be of the from `f(T)`, which returns a
        class which will be the instantation for type `T` of the given
        template.
        """
        template = cls(name, *args, **kwargs)
        param_list = [(T,) for T in template._T_list]

        def decorator(instantiation_func):

            def wrapped(param):
                T, = param
                return instantiation_func(T)

            template.add_instantiations(wrapped, param_list)
            return template

        return decorator

    def _on_add(self, param, cls):
        TemplateClass._on_add(self, param, cls)
        T, = param
        # Check that the user has not defined `__init__`, nad has defined
        # `_construct` and `_construct_copy`.
        if not issubclass(cls, LeafSystem_[T]):
            raise RuntimeError(
                "{} must inherit from {}".format(cls, LeafSystem_[T]))
        # Use the immediate `__dict__`, rather than querying the attributes, so
        # that we don't get spillover from inheritance.
        d = cls.__dict__
        no_init = "__init__" not in d
        has_custom_init = ("_construct" in d) and ("_construct_copy" in d)
        if not (no_init and has_custom_init):
            raise RuntimeError(
                "Convertible systems should not define `__init__`, but must "
                "define `_construct` and `_construct_copy` instead.")

        template = self

        def system_init(self, *args, **kwargs):
            converter = None
            if "converter" in kwargs:
                converter = kwargs.pop("converter")
            if converter is None:
                # Use default converter.
                converter = template._converter
            if template._check_if_copying(self, *args, **kwargs):
                cls._construct_copy(
                    self, *args, converter=converter, **kwargs)
            else:
                cls._construct(
                    self, *args, converter=converter, **kwargs)

        cls.__init__ = system_init

    def _check_if_copying(self, obj, *args, **kwargs):
        # Checks if a function signature implies a copy constructor.
        if len(args) >= 1:
            if self.is_subclass_of_instantiation(type(args[0])):
                return True
        return False

    def _make_converter(self):
        # Creates system scalar converter for the template class.
        converter = SystemScalarConverter()

        # Define capture to ensure the current values are bound, and do not
        # change through iteration.
        # N.B. This does not directly instantiation the template; it is
        # deferred to when the conversion is called.
        def add_captured(T_pair):
            T, U = T_pair

            def conversion(system):
                assert isinstance(system, self[U])
                return self[T](system)

            converter.Add[T, U](conversion)

        map(add_captured, self._T_pairs)
        return converter
