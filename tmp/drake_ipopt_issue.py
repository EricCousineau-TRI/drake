
import time

import numpy as np
from pydrake.math import eq
from pydrake.solvers import (
    CommonSolverOption,
    IpoptSolver,
    MathematicalProgram,
    SolverOptions,
)

N = 200
DATA =  np.linspace(0,100,N)#
# import casadi as ca

SOLVER_PD = IpoptSolver()
OPTS_PD = SolverOptions()
OPTS_PD.SetOption(CommonSolverOption.kPrintToConsole, 1)
OPTS_PD.SetOption(SOLVER_PD.id(), "print_level", 5)
OPTS_PD.SetOption(SOLVER_PD.id(), "print_user_options", "yes")
OPTS_PD.SetOption(SOLVER_PD.id(), "hessian_approximation", "limited-memory")
OPTS_PD.SetOption(SOLVER_PD.id(), "tol", 1e-8)
OPTS_PD.SetOption(SOLVER_PD.id(), "constr_viol_tol", 1e-4)
OPTS_PD.SetOption(SOLVER_PD.id(), "acceptable_tol", 1e-6)
OPTS_PD.SetOption(SOLVER_PD.id(), "acceptable_constr_viol_tol", 1e-2)
OPTS_PD.SetOption(SOLVER_PD.id(), "print_timing_statistics", "no")


OPTS_CD = {}
OPTS_CD["ipopt"] = {"print_level": 5,
                        "print_timing_statistics": "no",
                        "hessian_approximation": "limited-memory",
                        # hessian_approximation="exact",
                        "print_user_options": "yes",
                        "tol": 1e-8,
                        "constr_viol_tol": 1e-4,
                        "acceptable_tol": 1e-6,
                        "acceptable_constr_viol_tol": 1e-2,
                        "sb": "yes",
                        "linear_solver": "mumps"}

def prog_one_test():
    prog = MathematicalProgram()
    integral_val = prog.NewContinuousVariables(N + 1,"integral_val")
    controls = prog.NewContinuousVariables(N,"controls")
    max_val = prog.NewContinuousVariables(1,"max_val")

    prog.AddLinearCost(max_val[0]+ np.sum(integral_val)/N)
    prog.AddBoundingBoxConstraint(0,np.inf,max_val)
    prog.AddLinearConstraint(integral_val[0] == integral_val[-1])
    prog.AddLinearConstraint(integral_val[0] == 0)

    prog.AddBoundingBoxConstraint(0.0,100.0,integral_val)


    ##PROBLEM LINES
    prog.AddLinearConstraint(-max_val - controls ,np.ones_like(controls)*-np.inf, -DATA)
    prog.AddLinearConstraint(eq(integral_val[1:],( integral_val[:-1] + controls*.25)))

    start_t_pd = time.time()
    result_pd = SOLVER_PD.Solve(prog, solver_options=OPTS_PD)
    solve_t_pd = time.time() - start_t_pd

    return solve_t_pd,result_pd.GetSolution(controls), result_pd.GetSolution(integral_val), result_pd.GetSolution(max_val)


def prog_two_test():
    prog = MathematicalProgram()
    integral_val = prog.NewContinuousVariables(N + 1,"integral_val")
    controls = prog.NewContinuousVariables(N,"controls")
    max_val = prog.NewContinuousVariables(1,"max_val")

    prog.AddLinearCost(max_val[0] + np.sum(integral_val)/N)
    prog.AddBoundingBoxConstraint(0,np.inf,max_val)
    prog.AddLinearConstraint(integral_val[0] == integral_val[-1])
    prog.AddLinearConstraint(integral_val[0] == 0)

    prog.AddBoundingBoxConstraint(0.0,100.0,integral_val)


    #FIX PROLEM LINES
    for i in range(N):
        prog.AddLinearConstraint(integral_val[i+1]==integral_val[i] + controls[i]*.25)
        prog.AddLinearConstraint(-max_val- controls[i] ,[-np.inf], [-DATA[i]])

    start_t_pd = time.time()
    result_pd = SOLVER_PD.Solve(prog, solver_options=OPTS_PD)
    solve_t_pd = time.time() - start_t_pd

    return solve_t_pd, result_pd.GetSolution(controls), result_pd.GetSolution(integral_val), result_pd.GetSolution(max_val)


def casadi_test():

    controls = ca.SX.sym("x",N)
    integral_val = ca.SX.sym("soc",N+1)
    max_val = ca.SX.sym("max_grid_draw")

    w = [controls,integral_val,max_val]
    w0 = [np.zeros(N),np.zeros(N+1),np.max(DATA)]
    lbw = [-np.ones(N)*ca.inf,np.zeros(N+1),0]
    ubw = [np.ones(N)*ca.inf,np.ones(N+1)*160,ca.inf]

    g = [-controls - max_val]
    lbg = [-np.ones(N)*ca.inf]
    ubg = [-DATA]
    g += [integral_val[1:] - integral_val[:-1] - controls*.25]
    lbg += [np.zeros(N)]
    ubg += [np.zeros(N)]

    g += [integral_val[0] - integral_val[-1]]
    lbg += [0]
    ubg += [0]

    g += [integral_val[0]]
    lbg += [0]
    ubg += [0]

    g = ca.vertcat(*g)
    lbg = ca.vertcat(*lbg)
    ubg = ca.vertcat(*ubg)
    w = ca.vertcat(*w)
    w0 = ca.vertcat(*w0)
    lbw = ca.vertcat(*lbw)
    ubw = ca.vertcat(*ubw)

    f = max_val + ca.sum1(integral_val)/N
    nlp = {"x":w,"f":f,"g":g}

    solver = ca.nlpsol("solver","ipopt",nlp,OPTS_CD)
    start_t_pd = time.time()
    sol = solver(x0=w0,lbx=lbw,ubx=ubw,lbg=lbg,ubg=ubg)
    solve_t_pd = time.time() - start_t_pd
    x = sol["x"].full().flatten()

    sol_expander = ca.Function("expander",[w],[controls,integral_val,max_val],["x"],["controls","integral_val","max_val"])

    controls_opt,integral_val_opt,max_val_opt = sol_expander(x)

    return solve_t_pd,controls_opt,integral_val_opt,max_val_opt




def main():
    print("Starting test 1")
    solve_t_pd, controls_pd, integral_val_pd, max_val_pd = prog_one_test()
    print("Solve time: ", solve_t_pd)
    print("-------------------------------------------")
    print("Starting test 2")
    solve_t_pd2, controls_pd2, integral_val_pd2, max_val_pd2 = prog_two_test()


    print("-------------------------------------------")
    controls_norm = np.linalg.norm(controls_pd - controls_pd2)
    integral_val_norm = np.linalg.norm(integral_val_pd - integral_val_pd2)
    max_val_norm = np.linalg.norm(max_val_pd - max_val_pd2)
    print("-------------------------------------------")

    # print("-------------------------------------------")
    # print("Casadi Sol")
    # solve_t_pd3, controls_pd3, integral_val_pd3, max_val_pd3 = casadi_test()

    print("Solve time: ", solve_t_pd)
    print("Solve time 2 : ", solve_t_pd2)
    # print("Casadi Solve Time:", solve_t_pd3)
    print("Controls norm: ", controls_norm)
    print("Integral norm: ", integral_val_norm)
    print("Max norm: ", max_val_norm)

    # print("Controls norm casadi: ", np.linalg.norm(controls_pd - controls_pd3))
    # print("Integral norm casadi: ", np.linalg.norm(integral_val_pd - integral_val_pd3))
    # print("Max norm casadi: ", np.linalg.norm(max_val_pd - max_val_pd3))







if __name__ == "__main__":
    main()




