import numpy as np
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.math import RigidTransform
from pydrake.geometry import HalfSpace, ProximityProperties


def main():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-4)

    # contact parameters
    friction = CoulombFriction(0.18, 0.18)
    props = ProximityProperties()
    props.AddProperty("material", "point_contact_stiffness", 1.0e4)
    props.AddProperty("material", "hunt_crossley_dissipation", 0.5)
    props.AddProperty("material", "coulomb_friction", friction)

    X_WG = RigidTransform()

    ## Update model file location here ##
    Parser(plant).AddModelFromFile("tmp/cube.urdf")

    plant.RegisterCollisionGeometry(plant.world_body(), X_WG, HalfSpace(), "collision", props)

    plant.Finalize()
    plant.set_stiction_tolerance(1e-3)

    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    sim = Simulator(diagram)
    sim.Initialize()

    sim.AdvanceTo(0.2)


assert __name__ == '__main__'
main()
