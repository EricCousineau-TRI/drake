"""
$ bazel build //tmp:repro

$ bazel-bin/tmp/repro --num_x=1
Time / iter: 0.0047 ms
$ bazel-bin/tmp/repro --num_x=1 --disable_scipy
Time / iter: 0.0048 ms
$ bazel-bin/tmp/repro --num_x=2
Time / iter: 0.2794 ms
$ bazel-bin/tmp/repro --num_x=2 --disable_scipy
Time / iter: 0.0210 ms
$ bazel-bin/tmp/repro --num_x=13
Time / iter: 0.2816 ms
$ bazel-bin/tmp/repro --num_x=13 --disable_scipy
Time / iter: 0.0257 ms

"""

import argparse
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_x", type=int, required=True)
    parser.add_argument("--disable_scipy", action="store_true")
    args = parser.parse_args()

    if args.disable_scipy:
        import sys
        sys.modules["scipy"] = None

    import numpy as np
    from numpy import array

    from pydrake.solvers import MathematicalProgram

    prog = MathematicalProgram()

    num_x = args.num_x
    x = prog.NewContinuousVariables(num_x, "x")
    # ^ this is np.array(dtype=object) of Variable
    A = np.eye(num_x)
    b = np.ones(num_x)

    count = 100
    t_start = time.time()
    for i in range(count):
        prog.AddLinearConstraint(A, -b, b, x)
    dt_mean = (time.time() - t_start) / count
    print(f"Time / iter: {dt_mean * 1000:.4f} ms")


if __name__ == "__main__":
    main()
