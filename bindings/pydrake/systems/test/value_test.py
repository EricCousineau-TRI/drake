#!/usr/bin/env python
# -*- coding: utf8 -*-

from __future__ import print_function

import copy
import unittest
import numpy as np

from pydrake.systems.framework import (
    AbstractValue,
    BasicVector,
    Value,
    )
# from pydrake.systems.test.test_util import MoveOnlyValue
from pydrake.systems.test.test_util import (
    DeleteListenerSystem,
)


def pass_through(x):
    return x


# TODO(eric.cousineau): Add negative (or positive) test cases for AutoDiffXd
# and Symbolic once they are in the bindings.


class TestValue(unittest.TestCase):
    def test_basic_vector_double(self):
        # Test constructing vectors of sizes [0, 1, 2], and ensure that we can
        # construct from both lists and `np.array` objects with no ambiguity.
        for n in [0, 1, 2]:
            for wrap in [pass_through, np.array]:
                # Ensure that we can get vectors templated on double by
                # reference.
                expected_init = wrap(map(float, range(n)))
                expected_add = wrap([x + 1 for x in expected_init])
                expected_set = wrap([x + 10 for x in expected_init])

                value_data = BasicVector(expected_init)
                value = value_data.get_mutable_value()
                self.assertTrue(np.allclose(value, expected_init))

                # Add value directly.
                # TODO(eric.cousineau): Determine if there is a way to extract
                # the pointer referred to by the buffer (e.g. `value.data`).
                value[:] += 1
                self.assertTrue(np.allclose(value, expected_add))
                self.assertTrue(
                    np.allclose(value_data.get_value(), expected_add))
                self.assertTrue(
                    np.allclose(value_data.get_mutable_value(), expected_add))

                # Set value from `BasicVector`.
                value_data.SetFromVector(expected_set)
                self.assertTrue(np.allclose(value, expected_set))
                self.assertTrue(
                    np.allclose(value_data.get_value(), expected_set))
                self.assertTrue(
                    np.allclose(value_data.get_mutable_value(), expected_set))
                # Ensure we can construct from size.
                value_data = BasicVector(n)
                self.assertEquals(value_data.size(), n)

    def test_abstract_value_copyable(self):
        expected = "Hello world"
        value = Value[str](expected)
        self.assertTrue(isinstance(value, AbstractValue))
        self.assertEquals(value.get_value(), expected)
        expected_new = "New value"
        value.set_value(expected_new)
        self.assertEquals(value.get_value(), expected_new)

    def test_abstract_value_move_only(self):
        x = MoveOnlyValue(10)
        # This will maintain a reference.
        value = Value[MoveOnlyValue](x)
        x.set_value(20)
        self.assertEquals(value.get_value().get_value(), 20)
        # Set value.
        value.get_mutable_value().set_value(30)
        self.assertEquals(value.get_value().get_value(), 30)

    def test_abstract_value_py_object(self):
        expected = {"x": 10}
        value = Value[object](expected)
        # Value is by reference, *not* by copy.
        self.assertTrue(value.get_value() is expected)
        # Update mutable version.
        value.get_mutable_value()["y"] = 30
        self.assertEquals(value.get_value(), expected)
        # Cloning the value should perform a shallow copy of the Python object.
        value_clone = copy.copy(value)
        self.assertEquals(value_clone.get_value(), expected)
        self.assertTrue(value_clone.get_value() is not expected)
        # Using `set_value` on the original value changes object reference.
        expected_new = {"a": 20}
        value.set_value(expected_new)
        self.assertEquals(value.get_value(), expected_new)
        self.assertTrue(value.get_value() is not expected)

    def test_abstract_value_make(self):
        value = AbstractValue.Make("Hello world")
        self.assertTrue(isinstance(value, Value[str]))
        value = AbstractValue.Make(MoveOnlyType(10))
        self.assertTrue(isinstance(value, Value[MoveOnlyType]))
        value = AbstractValue.Make({"x": 10})
        self.assertTrue(isinstance(value, Value[object]))


if __name__ == '__main__':
    unittest.main()
