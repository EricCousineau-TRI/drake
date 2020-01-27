import numpy as np
import unittest

import convert_py as mut


def make_issue(nx, ny, nu):
    print(f"make_issue(nx={nx}, ny={ny}, nu={nu})")
    A = np.zeros((nx, nx))
    B = np.zeros((nx, nu))
    C = np.zeros((ny, nx))
    D = np.zeros((ny, nu))
    mut.pass_through(A)
    mut.pass_through(B)
    mut.pass_through(C)
    mut.pass_through(D)


class TestConvert(unittest.TestCase):
    def test_convert(self):
        mut.pass_through(np.zeros((0, 1)))
        mut.pass_through(np.zeros((0, 2)))

    def test_make_issue(self):
        make_issue(nx=0, ny=1, nu=1)
        make_issue(nx=0, nu=2, ny=2)
