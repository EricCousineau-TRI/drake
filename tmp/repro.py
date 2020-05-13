from pydrake.systems.framework import LeafSystem, Diagram
from drake.tmp.cc import make_my_diagram_as_system, make_my_diagram_as_diagram, as_diagram

system = make_my_diagram_as_system()
print(system)
print(as_diagram(system))
assert as_diagram(system) is not system  # :(
diagram = make_my_diagram_as_diagram()
print(diagram)
