import os

import numpy as np
from pydrake.all import VectorSystem, LinearSystem, LinearQuadraticRegulator, DiagramBuilder, Simulator

def main():
    A = np.array([[0., 1.], [0., 0.]])
    B = np.array([[0.], [1.]])
    C = np.eye(2)
    D = np.zeros((2, 1))
    plant = LinearSystem(A, B, C, D)

    controller = LinearQuadraticRegulator(plant, np.eye(2), np.eye(1))

    ## UNCOMMENT HERE TO MAKE IT WORK
    # class DoubleIntegrator(VectorSystem):
    #     def __init__(self):
    #         VectorSystem.__init__(self, 1, 2, direct_feedthrough=False)
    #         self.DeclareContinuousState(2)
    #     def DoCalcVectorTimeDerivatives(self, context, u, x, xdot):
    #         xdot[0] = x[1]
    #         xdot[1] = u
    #     def DoCalcVectorOutput(self, context, u, x, y):
    #         y[:] = x
    # plant = DoubleIntegrator()

    builder = DiagramBuilder()
    plant = builder.AddSystem(plant)
    controller = builder.AddSystem(controller)
    builder.Connect(controller.get_output_port(0), plant.get_input_port(0))
    builder.Connect(plant.get_output_port(0), controller.get_input_port(0))
    diagram = builder.Build()

    print(controller.A())
    print(controller.B())
    print(controller.C())
    print(controller.D())

    simulator = Simulator(diagram)
    simulator.get_mutable_context().SetContinuousState([0., 0.])
    simulator.AdvanceTo(1.)

    print("Done")


assert __name__ == "__main__"
assert os.environ["PYTHONUNBUFFERED"] == "1", (
    "Run with:   env PYTHONUNBUFFERED=1")
# main()
import sys, trace
sys.stdout = sys.stderr
tracer = trace.Trace(trace=1, count=0, ignoredirs=["/usr", sys.prefix])
tracer.runfunc(main)
