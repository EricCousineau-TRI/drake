# from https://drakedevelopers.slack.com/archives/C2CK4CWE7/p1584994164030500

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, LeafSystem, BasicVector
from pydrake.systems.primitives import ConstantVectorSource, Gain


class Sink(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self.input_port = self.DeclareVectorInputPort("u", BasicVector(1))

    def DoPublish(self, context, events):
        u = self.input_port.Eval(context)
        print(f"Hey {u}")


def main():
    builder = DiagramBuilder()
    source = builder.AddSystem(ConstantVectorSource([None]))
    sink = builder.AddSystem(Sink())
    builder.Connect(source.get_output_port(0), sink.input_port)

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    diagram.Publish(context)


# main()
import sys, trace
sys.stdout = sys.stderr
tracer = trace.Trace(trace=1, count=0, ignoredirs=["/usr", sys.prefix])
tracer.runfunc(main)
