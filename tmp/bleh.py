# from https://drakedevelopers.slack.com/archives/C2CK4CWE7/p1584994164030500

from pydrake.examples.pendulum import PendulumInput, PendulumPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource


def main():
    builder = DiagramBuilder()
    pendulum = builder.AddSystem(PendulumPlant())
    source = builder.AddSystem(ConstantVectorSource([PendulumInput().set_tau(0.)]))
    builder.Connect(source.get_output_port(0), pendulum.get_input_port())

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)

    simulator.Initialize()
    simulator.AdvanceTo(0.)


# main()
import sys, trace
sys.stdout = sys.stderr
tracer = trace.Trace(trace=1, count=0, ignoredirs=["/usr", sys.prefix])
tracer.runfunc(main)
