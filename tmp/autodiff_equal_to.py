# simplified user-friendly version of `numpy_compare.equal_to` for AutoDiffXd.

import numpy as np

from pydrake.autodiffutils import AutoDiffXd


@np.vectorize
def autodiff_equal_to(a, b):
    if a.value() == b.value():
        da = a.derivatives()
        db = b.derivatives()
        if da.shape == db.shape and (da == db).all():
            return True
    return False


def main():
    a = AutoDiffXd(1.0, [1.0, 2.0])
    b = AutoDiffXd(1.0, [3.0, 4.0])
    print(autodiff_equal_to(a, a))
    print(autodiff_equal_to([a, a], a))
    print(autodiff_equal_to(a, b))
    print(autodiff_equal_to([a], [b]))


assert __name__ == "__main__"
main()

"""
True
[ True  True]
False
[False]
"""
