"""Loads a model file (*.sdf or *.urdf) and then displays it in all
available visualizers.

This publishes the robot in its home configuration. To view the model while
manipulating joints, please see `geometry_inspector`.

Example usage:

    cd drake
    bazel build \
        //tools:drake_visualizer @meshcat_python//:meshcat-server \
        //manipulation/util:show_model

    # Terminal 1
    ./bazel-bin/tools/drake_visualizer
    # Terminal 2
    ./bazel-bin/external/meshcat_python/meshcat-server
    # Terminal 3 (wait for visualizers to start)
    ./bazel-bin/manipulation/util/show_model \
        --meshcat default \
        ./manipulation/models/iiwa_description/iiwa7/iiwa7_no_collision.sdf

"""

from __future__ import print_function
import argparse
import os
import sys

from pydrake.lcm import DrakeLcm
from pydrake.geometry import ConnectDrakeVisualizer, SceneGraph
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.multibody_tree.parsing import (
    AddModelFromSdfFile, AddModelFromUrdfFile)
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "filename", type=str,
        help="URDF or SDF file to display. If using under `bazel run`, "
             "please ensure you provide an absolute path.")
    MeshcatVisualizer.add_argparse_argument(parser)
    args = parser.parse_args()
    filename = args.filename

    # Construct Plant + SceneGraph.
    builder = DiagramBuilder()
    plant = builder.AddSystem(MultibodyPlant())
    scene_graph = builder.AddSystem(SceneGraph())
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    builder.Connect(
        scene_graph.get_query_output_port(),
        plant.get_geometry_query_input_port())
    builder.Connect(
        plant.get_geometry_poses_output_port(),
        scene_graph.get_source_pose_port(plant.get_source_id()))

    # Load the model file.
    if not os.path.isfile(filename):
        print("File does not exist: {}".format(filename), file=sys.stderr)
        if not os.path.isabs(filename):
            print("  File is not absolute; if you are running under "
                  "`bazel run`, please provide an absolute path.",
                  file=sys.stderr)
        sys.exit(1)
    ext = os.path.splitext(filename)[1]
    if ext == ".sdf":
        AddModelFromSdfFile(filename, plant)
    elif ext == ".urdf":
        AddModelFromUrdfFile(filename, plant)
    else:
        print("Unknown extension " + ext, file=sys.stderr)
        sys.exit(1)

    plant.Finalize()

    # Publish to Drake Visualizer.
    drake_viz_pub = ConnectDrakeVisualizer(builder, scene_graph)

    # Publish to Meshcat.
    meshcat_viz = builder.AddSystem(
        MeshcatVisualizer(scene_graph, zmq_url=args.meshcat))
    builder.Connect(scene_graph.get_pose_bundle_output_port(),
                    meshcat_viz.get_input_port(0))

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()

    # Use Simulator to dispatch initialization events.
    # TODO(eric.cousineau): Simplify as part of #10015.
    Simulator(diagram).Initialize()
    # Publish draw messages with current state.
    diagram.Publish(context)


if __name__ == '__main__':
    main()
