Commit's behavior:

```
$ bazel run //bindings/pydrake/solvers:py/mathematicalprogram_test -- TestMathematicalProgram.test_pycost_wrap_error TestMathematicalProgram.test_pyconstraint_wrap_error
...
pycost

<class 'float'>
bad input
  ERROR: PyFunctionCost: Input must be of .ndim = 1 or 2 (vector) and .size = 1. Got .ndim = 1 and .size = 2 instead.
bad output
  ERROR: PyFunctionCost: Output must be of .ndim = 0 (scalar) and .size = 1. Got .ndim = 1 and .size = 1 instead.

<class 'pydrake.autodiffutils.AutoDiffXd'>
bad input
  ERROR: PyFunctionCost: Input must be of .ndim = 1 or 2 (vector) and .size = 1. Got .ndim = 1 and .size = 2 instead.
bad output
  ERROR: PyFunctionCost: Output must be of .ndim = 0 (scalar) and .size = 1. Got .ndim = 1 and .size = 1 instead.
.

pyconstraint

<class 'float'>
bad input
  ERROR: PyFunctionConstraint: Input must be of .ndim = 1 or 2 (vector) and .size = 1. Got .ndim = 1 and .size = 2 instead.
bad output
  ERROR: PyFunctionConstraint: Output must be of .ndim = 1 or 2 (vector) and .size = 1. Got .ndim = 0 and .size = 1 instead.

<class 'pydrake.autodiffutils.AutoDiffXd'>
bad input
  ERROR: PyFunctionConstraint: Input must be of .ndim = 1 or 2 (vector) and .size = 1. Got .ndim = 1 and .size = 2 instead.
bad output
  ERROR: PyFunctionConstraint: Output must be of .ndim = 1 or 2 (vector) and .size = 1. Got .ndim = 0 and .size = 1 instead.
```
