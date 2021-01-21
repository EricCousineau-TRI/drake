"""
Provides an easy-to-use template (pattern matching) for C++/Python API
deprecation.

For more details, please review the following documentation:

- https://drake.mit.edu/code_review_checklist.html
- https://drake.mit.edu/doxygen_cxx/group__python__bindings.html#PydrakeDeprecation
- https://drake.mit.edu/doxygen_cxx/group__python__bindings.html#PydrakeDoc
"""  # noqa

import unittest

import deprecation_example as mut
import deprecation_example.cc_module as mut_cc


class TestDeprecationExample(unittest.TestCase):
    def test_example_cpp_struct(self):
        obj = mut_cc.ExampleCppStruct()
        self.assertEqual(obj.i, 0)
        self.assertEqual(obj.j, 0)

    def test_example_cpp_class_ctors(self):
        mut_cc.ExampleCppClass()
        mut_cc.ExampleCppClass(0)
        mut_cc.ExampleCppClass(0.0)

    def test_example_cpp_class_methods(self):
        obj = mut_cc.ExampleCppClass()
        obj.DeprecatedMethod()
        obj.overload()
        obj.overload(0)

    def test_example_cpp_class_properties(self):
        obj = mut_cc.ExampleCppClass()
        self.assertEqual(obj.deprecated_aliased_prop, 0)
