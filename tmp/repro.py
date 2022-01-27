import numpy as np

from pydrake.all import *
from drake.tmp.cc import addFlatTerrain


def addFlatTerrain_py(plant, scene_graph, mu_static, mu_kinetic, normal_W=np.array([0.0, 0, 1])):
    if not plant.geometry_source_is_registered():
        plant.RegisterAsSourceForSceneGraph(scene_graph)


    point_W = np.array([0.0, 0, 0])
    friction = CoulombFriction(mu_static, mu_kinetic)

    # A half-space for the ground geometry.
    X_WG = HalfSpace.MakePose(normal_W, point_W)

    plant.RegisterCollisionGeometry(plant.world_body(), X_WG, HalfSpace(),
                                   "collision", friction)

    # Add visual for the ground.
    plant.RegisterVisualGeometry(plant.world_body(), X_WG, HalfSpace(),
                                "visual", np.array([0.0, 0, 0, 1]))


def method_py():
    print("py")
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-4)
    addFlatTerrain_py(plant=plant, scene_graph=scene_graph, mu_static=1.0, mu_kinetic=1.0)
    plant.Finalize()
    DrakeVisualizer.AddToBuilder(builder, scene_graph)
    diagram = builder.Build()
    Simulator(diagram).AdvanceTo(0.1)


def method_cc():
    print("cc")
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-4)
    addFlatTerrain(plant=plant, scene_graph=scene_graph, mu_static=1.0, mu_kinetic=1.0)
    plant.Finalize()
    DrakeVisualizer.AddToBuilder(builder, scene_graph)
    diagram = builder.Build()
    Simulator(diagram).AdvanceTo(0.1)


def main():
    method_py()
    method_cc()
    print("[ Done ]")


assert __name__ == "__main__"
main()
