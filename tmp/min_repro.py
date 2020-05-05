from pydrake.all import MathematicalProgram, Solve, AutoDiffXd

prog = MathematicalProgram()
x = prog.NewContinuousVariables(1, 'x')

def constraint(x):
    return [AutoDiffXd(0.)]

constraint_binding = prog.AddConstraint(
    constraint, lb=[0.], ub=[0.], vars=x)
# Good
constraint_binding.evaluator().Eval([AutoDiffXd(0.)])
# BAD
constraint_binding.evaluator().Eval([0.])
