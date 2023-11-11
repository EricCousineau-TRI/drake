import unittest

import numpy as np
import scipy.sparse

from drake.tools.workspace.pybind11.test.eigen_dtype_object_cnoconvert import (
    AcceptMatrix,
    AcceptMatrixAndObject,
    MyScalar,
)


class Test(unittest.TestCase):
    def test_overload(self):
        A_dense = np.eye(2)
        A_sparse = scipy.sparse.csc_matrix(np.eye(2))
        self.assertEqual(AcceptMatrix(A_dense), "dense")
        self.assertEqual(AcceptMatrix(A_sparse), "sparse")

        x = np.array([MyScalar(1), MyScalar(2)])
        self.assertEqual(x.dtype, object)
        self.assertEqual(AcceptMatrixAndObject(A_dense, x), "dense")
        self.assertEqual(AcceptMatrixAndObject(A_sparse, x), "sparse")


if __name__ == "__main__":
    unittest.main()
