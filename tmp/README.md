# Repro

<https://github.com/RobotLocomotion/drake/issues/13181#issuecomment-623607589>

Run:

```sh
$ bazel run --config snopt //tmp:py/kneed_compass_gait
...
  File "...runfiles/drake/tmp/kneed_compass_gait.py", line 330,
in <module>
    infeasible = mp.GetInfeasibleConstraints(prog, result) #ERROR OCCURS HERE
RuntimeError: Unable to cast Python instance to C++ type (compile in debug mode for details)
```
