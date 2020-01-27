import numpy as np
import unittest

import convert_py as mut


class TestConvert(unittest.TestCase):
    def test_convert(self):
        mut.pass_through(np.zeros((0, 1)))
        mut.pass_through(np.zeros((0, 2)))
