# From: https://github.com/RobotLocomotion/drake/issues/13181#issuecomment-623607589
import numpy as np
import matplotlib.pyplot as plt
from IPython.display import HTML

# drake
from pydrake.all import (MultibodyPlant, Parser, DiagramBuilder, Simulator,
                         PlanarSceneGraphVisualizer, SceneGraph, TrajectorySource,
                         SnoptSolver, MultibodyPositionToGeometryPose, PiecewisePolynomial,
                         MathematicalProgram, JacobianWrtVariable, eq, GetInfeasibleConstraints)
from pydrake import symbolic
from pydrake import math
from underactuated import FindResource

v = 1.5

# friction coefficient between feet and ground
friction = .2

# position of the feet in the respective leg frame
# (must match the urdf)
foot_in_leg = {
    'stance_leg': np.zeros(3),        # stance foot in stance-leg frame
    'swing_leg': np.array([0, 0, -1]) # swing foot in swing-leg frame
}

# parse urdf and create the MultibodyPlant
kneed_compass_gait = MultibodyPlant(time_step=0)
file_name = 'kneed_compass_gait.urdf'
Parser(kneed_compass_gait).AddModelFromFile(file_name)
kneed_compass_gait.Finalize()

# overwrite MultibodyPlant with its autodiff copy
kneed_compass_gait = kneed_compass_gait.ToAutoDiffXd()

# number of configuration variables
nq = kneed_compass_gait.num_positions()

# number of components of the contact forces
nf = 2

# number of actuators (There are three, but only two of them are used at each step. The knee actuator at the stance leg is not used)
na = 3

# Function that given the current configuration, velocity,
# acceleration, and contact force at the stance foot, evaluates
# the manipulator equations. The output of this function is a
# vector with dimensions equal to the number of configuration
# variables. If the output of this function is equal to zero
# then the given arguments verify the manipulator equations.
def manipulator_equations(vars):
    
    # split input vector in subvariables
    # configuration, velocity, acceleration, stance-foot force


    assert vars.size == 3 * nq + nf + na 
    # (number_of_derivatives + 1)*(x,y,q1,q2,q3) 
    # + number of forces on stance foot(Fx, Fy) + number of actuators (3)

    #We split the vector at certain locations so that we get:
    #[0:nq-1], [nq:2*nq-1], [2nq:3*nq-1] and so on
    split_at = [nq, 2 * nq, 3 * nq, 3*nq + nf]
    q, qd, qdd, f, u = np.split(vars, split_at)
    
    # set compass gait state
    context = kneed_compass_gait.CreateDefaultContext()
    kneed_compass_gait.SetPositions(context, q)
    kneed_compass_gait.SetVelocities(context, qd)
    
    # matrices for the manipulator equations
    M = kneed_compass_gait.CalcMassMatrixViaInverseDynamics(context)
    Cv = kneed_compass_gait.CalcBiasTerm(context)
    tauG = kneed_compass_gait.CalcGravityGeneralizedForces(context)
    
    # Jacobian of the stance foot
    J = get_foot_jacobian(kneed_compass_gait, context, 'stance_leg')
    
    U = np.array([0, 0, 0, u[1], u[2]])
    ### The actuators should be added here somewhere

    # return violation of the manipulator equations
    return M.dot(qdd) + Cv - tauG - J.T.dot(f) - U

# Function that given the current configuration, returns
# the distance of the swing foot from the ground (scalar).
# We have penetration if the function output is negative.
def swing_foot_height(q):
    
    # get reference frames for the swing leg and the ground
    leg_frame = kneed_compass_gait.GetBodyByName('swing_leg').body_frame()
    ground_frame = kneed_compass_gait.GetBodyByName('ground').body_frame()
    
    # position of the swing foot in ground coordinates
    context = kneed_compass_gait.CreateDefaultContext()
    kneed_compass_gait.SetPositions(context, q)
    swing_foot_position = kneed_compass_gait.CalcPointsPositions(
        context,
        leg_frame,
        foot_in_leg['swing_leg'],
        ground_frame
    )
    
    # return only the coordinate z
    # (distance normal to the ground)
    return swing_foot_position[-1]

# Function that implements the impulsive collision derived in
# the textbook appendix. Arguments are: compass gait configuration,
# velocities before and after heel strike, and the swing-foot
# impulse (in latex, $\int_{t_c^-}^{t_c^+} \lambda dt$).
# Returns a vector of quantities that must vanish in order
# for the impulsive dynamics to be verified: it enforces the velocity
# jump due to the impulse, and the inelastic behavior of the
# collision (zero coefficient of restitution $e$).
# See http://underactuated.mit.edu/multibody.html#impulsive_collision
def reset_velocity_heelstrike(vars):
    
    # split input vector in subvariables
    # qd_pre: generalized velocity before the heel strike
    # qd_post: generalized velocity after the heel strike
    # imp: swing-foot collision impulse (2d vector)
    assert vars.size == 3 * nq + nf 
    split_at = [nq, 2 * nq, 3 * nq]
    q, qd_pre, qd_post, imp = np.split(vars, split_at)

    # set compass gait configuration
    context = kneed_compass_gait.CreateDefaultContext()
    kneed_compass_gait.SetPositions(context, q)
    
    # get necessary matrices
    M = kneed_compass_gait.CalcMassMatrixViaInverseDynamics(context)
    J = get_foot_jacobian(kneed_compass_gait, context, 'swing_leg')
    
    # return a vector that must vanish for the impulsive dynamics to hold
    return np.concatenate((
        M.dot(qd_post - qd_pre) - J.T.dot(imp), # velocity jump due to the impulse
        J.dot(qd_post)                          # zero velocity restitution
    ))

# Function that given a leg, returns the Jacobian matrix for the related foot.
def get_foot_jacobian(compass_gait, context, leg):
    
    # get reference frames for the given leg and the ground
    leg_frame = kneed_compass_gait.GetBodyByName(leg).body_frame()
    ground_frame = kneed_compass_gait.GetBodyByName('ground').body_frame()

    # compute Jacobian matrix
    J = kneed_compass_gait.CalcJacobianTranslationalVelocity(
        context,
        JacobianWrtVariable(0),
        leg_frame,
        foot_in_leg[leg],
        ground_frame,
        ground_frame
    )
    
    # discard y components since we are in 2D
    return J[[0, 2]]

# time steps in the trajectory optimization
T = 50

# minimum and maximum time interval is seconds
h_min = .005
h_max = .05


# initialize program
prog = MathematicalProgram()

# vector of the time intervals
# (distances between the T + 1 break points)
h = prog.NewContinuousVariables(rows=T, cols=1, name='h')

# system configuration, generalized velocities, and accelerations
q = prog.NewContinuousVariables(rows=T+1, cols=nq, name='q')
qd = prog.NewContinuousVariables(rows=T+1, cols=nq, name='qd')
qdd = prog.NewContinuousVariables(rows=T, cols=nq, name='qdd')

# stance-foot force
f = prog.NewContinuousVariables(rows=T, cols=nf, name='f')

# heel strike impulse for the swing leg
imp = prog.NewContinuousVariables(nf, name='imp')

# generalized velocity after the heel strike
# (if "kneemirrored", this velocity must coincide with the
# initial velocity qd[0] to ensure periodicity)
qd_post = prog.NewContinuousVariables(nq, name='qd_post')


# The length of the stride is a decision variable. This is necessary because we will have constraints on the velocity
d = prog.NewContinuousVariables(1, name='d')
d_min = 0.01
d_max = 3.0
prog.AddBoundingBoxConstraint([d_min], [d_max], d).evaluator().set_description('d bounding box')

# The input vector is a decision variable
u = prog.NewContinuousVariables(rows=T, cols=na, name='u')
u_min = -5.0
u_max = 5.0
#prog.AddBoundingBoxConstraint([[u_min] * na]*T, [[u_max] * na]*T, u)
prog.AddBoundingBoxConstraint([u_min] * T, [u_max] * T, u[:,0]).evaluator().set_description('tau1 bounding box')
prog.AddBoundingBoxConstraint([u_min] * T, [u_max] * T, u[:,1]).evaluator().set_description('tau2 bounding box')
prog.AddBoundingBoxConstraint([u_min] * T, [u_max] * T, u[:,2]).evaluator().set_description('tau3 bounding box')

# lower and upper bound on the time steps for all t
prog.AddBoundingBoxConstraint([h_min] * T, [h_max] * T, h).evaluator().set_description('h bounding box')

# link the configurations, velocities, and accelerations
# uses implicit Euler method, https://en.wikipedia.org/wiki/Backward_Euler_method
### Constraints for propagation of the system
for t in range(T):
    prog.AddConstraint(eq(q[t+1], q[t] + h[t] * qd[t+1])).evaluator().set_description('qd propagation ' + str(t))
    prog.AddConstraint(eq(qd[t+1], qd[t] + h[t] * qdd[t])).evaluator().set_description('qdd propagation ' + str(t))

# manipulator equations for all t (implicit Euler)
### Ensures that the system is calculated correctly
for t in range(T):
    vars = np.concatenate((q[t+1], qd[t+1], qdd[t], f[t], u[t]))
    prog.AddConstraint(manipulator_equations, lb=[0]*nq, ub=[0]*nq, vars=vars).evaluator().set_description('implicit euler ' + str(t))
    
# velocity reset across heel strike
# see http://underactuated.mit.edu/multibody.html#impulsive_collision
vars = np.concatenate((q[-1], qd[-1], qd_post, imp))
prog.AddConstraint(reset_velocity_heelstrike, lb=[0]*(nq+nf), ub=[0]*(nq+nf), vars=vars).evaluator().set_description('heel strike')
    
# mirror initial and final configuration
# see "The Walking Cycle" section of this notebook
prog.AddLinearConstraint(eq(q[0], - q[-1])).evaluator().set_description('initial mirror')

# mirror constraint between initial and final velocity
# see "The Walking Cycle" section of this notebook
prog.AddLinearConstraint(qd[0, 0] == 0).evaluator().set_description('qd[0,0]')
prog.AddLinearConstraint(qd[0, 1] == 0).evaluator().set_description('qd[0, 1]')
prog.AddLinearConstraint(qd[0, 2] == qd_post[2] + qd_post[3]).evaluator().set_description('qd[0, 2]')
prog.AddLinearConstraint(qd[0, 3] == - qd_post[3]).evaluator().set_description('qd[0, 3]')

# Constraints for achieving a desired velocity

#Making sure the stride length, d, corresponds to the angles at the beginning/end
prog.AddConstraint(eq(2*symbolic.sin(q[0, 2]), np.array([d/2]))).evaluator().set_description('Step length = initial stride')

# The time used for one stride multiplied by the desired velocity equals the stride length
prog.AddConstraint(eq(sum(h)*v, np.array([d]))).evaluator().set_description('T*V=d')

z = np.array([0])
zero = 0

# Making sure the stance foot is on the ground
prog.AddLinearConstraint(eq(q[:,0], [zero]*(T+1))).evaluator().set_description('x = 0')
prog.AddLinearConstraint(eq(q[:,1], [zero]*(T+1))).evaluator().set_description('y = 0')

# Making sure the swing foot starts on the ground (the legs are straight at t = 0 and t = T)
prog.AddLinearConstraint(q[0, 2] == -q[0, 3]/2).evaluator().set_description('swing leg z(0) = 0')

# The swing knee should be straight at t = 0
prog.AddLinearConstraint(q[0, 4] == z).evaluator().set_description('initial swing knee straight')



# Attempt to ensure the swing knee leg doesn't bend forward. Should be replaced by constraint forces
for t in range(T):
  prog.AddLinearConstraint(q[t, 4] <= z).evaluator().set_description('theta3 < 0 at t '+ str(t))





# Ensuring that the stance-foot contact force is in the friction cone for all times
for t in range(T):
  prog.AddLinearConstraint(f[t,1] >= z).evaluator().set_description('fz >= 0 at t = ' + str(t))
  prog.AddConstraint(f[t,0]**2 <= (f[t,1]*friction)**2).evaluator().set_description('horizontal friction at t = ' + str(t))

# Ensuring that the swing-foot impulse in the friction cone
prog.AddLinearConstraint(imp[1] >= z).evaluator().set_description('positive z impulse')
prog.AddConstraint(imp[0]**2 <= (imp[1]*friction)**2).evaluator().set_description('x impulse')


def powerCost(vars):
  #na qd_i, na u_i, h_i, d
  assert vars.size == 2*na + 1 + 1
  split_at = [na, 2*na, 2*na + 1]
  qd, u, h, d = np.split(vars, split_at)
  #return np.abs(qd.dot(u)*h/d)
  return np.abs(np.array([qd.dot(u)*h/d]))

for t in range(T):
  vars = np.concatenate((qd[t, 2:], u[t], h[t], d))
  #prog.AddCost(powerCost, vars=vars)


# vector of the initial guess
initial_guess = np.empty(prog.num_vars())

# initial guess for the time step
h_guess = h_max
prog.SetDecisionVariableValueInVector(h, [h_guess] * T, initial_guess)

# linear interpolation of the configuration
q0_guess = np.array([0, 0, .15, -.3, 0])
q_guess_poly = PiecewisePolynomial.FirstOrderHold(
    [0, T * h_guess],
    np.column_stack((q0_guess, - q0_guess))
)
qd_guess_poly = q_guess_poly.derivative()
qdd_guess_poly = q_guess_poly.derivative()

# set initial guess for configuration, velocity, and acceleration
q_guess = np.hstack([q_guess_poly.value(t * h_guess) for t in range(T + 1)]).T
qd_guess = np.hstack([qd_guess_poly.value(t * h_guess) for t in range(T + 1)]).T
qdd_guess = np.hstack([qdd_guess_poly.value(t * h_guess) for t in range(T)]).T
prog.SetDecisionVariableValueInVector(q, q_guess, initial_guess)
prog.SetDecisionVariableValueInVector(qd, qd_guess, initial_guess)
prog.SetDecisionVariableValueInVector(qdd, qdd_guess, initial_guess)

# initial guess for the normal component of the stance-leg force
bodies = ['body', 'stance_leg', 'swing_leg']
mass = sum(kneed_compass_gait.GetBodyByName(body).default_mass() for body in bodies)
g = - kneed_compass_gait.gravity_field().gravity_vector()[-1]
weight = mass * g
prog.SetDecisionVariableValueInVector(f[:, 1], [weight] * T, initial_guess)

# solve mathematical program with initial guess
solver = SnoptSolver()
result = solver.Solve(prog, initial_guess) ## The error occurs here if the cost function with np.abs() is added. math.abs() from pydrake threw an error related to the input type
if not result.is_success():
    infeasible = GetInfeasibleConstraints(prog, result) #ERROR OCCURS HERE
    print("Infeasible constraints:")
    for i in range(len(infeasible)):
        print(infeasible[i])

# ensure solution is found
print(f'Solution found? {result.is_success()}.')