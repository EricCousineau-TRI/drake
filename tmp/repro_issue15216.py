# https://github.com/RobotLocomotion/drake/issues/15216
from drake.tmp import debug
if __name__ == "__main__":
    debug.reexecute_if_unbuffered()

from pydrake.all import RigidTransform_, MathematicalProgram, Expression


@debug.traced
def main():
    prog = MathematicalProgram()
    p_A = prog.NewContinuousVariables(3)
    X_WA = RigidTransform_[Expression]()

    print(X_WA.multiply([0,0,0]))
    print(X_WA.multiply(p_A))  # <-- this line segfaults
    print("Done")


if __name__ == "__main__":
    main()
