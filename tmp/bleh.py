# from https://drakedevelopers.slack.com/archives/C2CK4CWE7/p1584994164030500

import numpy as np
from pydrake.all import Simulator, Integrator, ConstantVectorSource, DiagramBuilder, SignalLogger


def main():
    builder = DiagramBuilder()
    u = np.nan
    n = 1
    source = builder.AddSystem(ConstantVectorSource([u]))
    integrator = builder.AddSystem(Integrator(n))
    logger = builder.AddSystem(SignalLogger(n))
    builder.Connect(
        source.get_output_port(0), integrator.get_input_port(0))
    builder.Connect(
        integrator.get_output_port(0), logger.get_input_port(0))
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.AdvanceTo(0.)
    print("Done")


assert __name__ == "__main__"
main()
