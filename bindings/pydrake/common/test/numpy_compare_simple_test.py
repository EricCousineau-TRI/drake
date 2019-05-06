import unittest

import numpy as np

import pydrake.common.test_utilities.numpy_compare as npc

# Hack into private API to register custom comparisons.
class Custom(object):
    def __init__(self, value):
        assert isinstance(value, str)
        self._str = value

    def __str__(self):
        return self._str

    def assert_eq(self, other):
        assert self._str == other._str, (self, other)

    def assert_ne(self, other):
        assert self._str != other._str, (self, other)


npc._to_float[Custom] = lambda x: float(str(x))
npc._register_comparator(Custom, Custom, Custom.assert_eq, Custom.assert_ne)
npc._register_comparator(Custom, str, npc._str_eq, npc._str_ne)


class TestNumpyCompareSimple(unittest.TestCase):
    def test_to_float(self):
        # Scalar.
        xi = 1
        xf = npc.to_float(xi)
        self.assertEqual(xf.dtype, float)
        self.assertEqual(xi, xf)
        # Array.
        Xi = np.array([1, 2, 3], np.int)
        Xf = npc.to_float(Xi)
        self.assertEqual(Xf.dtype, float)
        np.testing.assert_equal(Xi, Xf)
        # Custom.
        a = Custom('1.')
        b = Custom('2.')
        self.assertEqual(npc.to_float(a), 1.)
        A = np.array([a, b])
        np.testing.assert_equal(npc.to_float(A), [1., 2.])

    def test_asserts_builtin(self):
        a = 1.
        b = 0.
        # Scalar.
        npc.assert_equal(a, a)
        with self.assertRaises(AssertionError):
            npc.assert_equal(a, b)
        npc.assert_not_equal(a, b)
        with self.assertRaises(AssertionError):
            npc.assert_not_equal(a, a)
        # Array.
        A = np.array([a, a])
        C = np.array([1., 2.])
        npc.assert_equal(A, a)
        npc.assert_equal(C, C)
        with self.assertRaises(AssertionError):
            npc.assert_equal(A, b)
        npc.assert_not_equal(A, A + [0, 0.1])
        npc.assert_not_equal(A, b)
        with self.assertRaises(AssertionError):
            npc.assert_not_equal(C, C)

    def test_asserts_custom(self):
        a = Custom('a')
        b = Custom('b')
        # Scalar.
        npc.assert_equal(a, a)
        npc.assert_equal(a, 'a')
        with self.assertRaises(AssertionError):
            npc.assert_equal(a, b)
        npc.assert_not_equal(a, b)
        with self.assertRaises(AssertionError):
            npc.assert_not_equal(a, a)
        with self.assertRaises(AssertionError):
            npc.assert_not_equal(a, 'a')
        # Array.
        A = np.array([a, a])
        C = np.array([Custom('c0'), Custom('c1')])
        npc.assert_equal(A, a)
        npc.assert_equal(A, 'a')
        npc.assert_equal(C, C)
        npc.assert_equal(C, ['c0', 'c1'])
        with self.assertRaises(AssertionError):
            npc.assert_equal(A, b)
        npc.assert_not_equal(A, [a, b])
        npc.assert_not_equal(A, ['a', 'b'])
        with self.assertRaises(AssertionError):
            npc.assert_not_equal(C, C)
