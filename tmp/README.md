Commit's behavior:

```
$ bazel run //bindings/pydrake/solvers:py/mathematicalprogram_test -- TestMathematicalProgram.test_pycost_wrap_error TestMathematicalProgram.test_pyconstraint_wrap_error
...
pycost

<class 'float'>
bad input
  ERROR: Unable to cast Python instance to C++ type (compile in debug mode for details)
bad output
  no error

<class 'pydrake.autodiffutils.AutoDiffXd'>
bad input
  ERROR: Unable to cast Python instance to C++ type (compile in debug mode for details)
bad output
  ERROR: Unable to cast Python instance to C++ type (compile in debug mode for details)
.

pyconstraint

<class 'float'>
bad input
  ERROR: Unable to cast Python instance to C++ type (compile in debug mode for details)
bad output
  ERROR: Unable to cast Python instance to C++ type (compile in debug mode for details)

<class 'pydrake.autodiffutils.AutoDiffXd'>
bad input
  ERROR: Unable to cast Python instance to C++ type (compile in debug mode for details)
bad output
  ERROR: Unable to cast Python instance to C++ type (compile in debug mode for details)
```
