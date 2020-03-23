# from https://drakedevelopers.slack.com/archives/C2CK4CWE7/p1584994164030500

import numpy as np
from pydrake.all import (
    DiagramBuilder, ConstantVectorSource, Integrator, Simulator)

builder = DiagramBuilder()
source = builder.AddSystem(ConstantVectorSource([np.nan]))
integrator = builder.AddSystem(Integrator(1))
builder.Connect(source.get_output_port(0), integrator.get_input_port(0))
diagram = builder.Build()
simulator = Simulator(diagram)
simulator.AdvanceTo(0.)  # loops forever
print("Done")
