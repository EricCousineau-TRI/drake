# https://github.com/RobotLocomotion/drake/issues/15216
from drake.tmp import debug
if __name__ == "__main__":
    debug.reexecute_if_unbuffered()

import numpy as np
from pydrake.all import RigidTransform_, Expression, Variable


@debug.traced
def main():
    p_A_var = np.array([Variable("x0"), Variable("x1"), Variable("x2")])
    to_sym = np.vectorize(Expression)
    p_A_sym = to_sym(p_A_var)

    X_WA = RigidTransform_[Expression]()

    # print(X_WA.multiply(p_A_sym))  # <-- now this line segfaults
    print(X_WA.multiply(p_A_var))
    print("Done")


if __name__ == "__main__":
    main()
