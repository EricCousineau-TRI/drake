# https://gist.github.com/mpetersen94/8c86de2c5769e07049de4fb13232da80
from pydrake.geometry import (Box, GeometryFrame, GeometryInstance,
                              MakePhongIllustrationProperties, SceneGraph)
from pydrake.math import RigidTransform
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer

builder = DiagramBuilder()
scene_graph = builder.AddSystem(SceneGraph())
meshcat = builder.AddSystem(MeshcatVisualizer(scene_graph, zmq_url="new"))
builder.Connect(scene_graph.get_pose_bundle_output_port(), meshcat.get_input_port(0))

source_id = scene_graph.RegisterSource()
frame_id = scene_graph.RegisterFrame(source_id, GeometryFrame("body"))
geom_instance = GeometryInstance(RigidTransform(), Box(1., 1., 1.), "box")
geom_id = scene_graph.RegisterAnchoredGeometry(source_id, geom_instance)
scene_graph.AssignRole(source_id, geom_id, MakePhongIllustrationProperties([0.5, 0.5, 0.5, 1.]))

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.Initialize()
