from textwrap import indent
import time

import numpy as np

from pydrake.all import (
    DiagramBuilder, AddMultibodyPlantSceneGraph, Parser, Simulator, ConnectDrakeVisualizer,
    JointIndex,
)

# Model setup:
# * Base link
# * Drawer 1: 0.1m from bottom, prismatic x and z (w/ spring)
# * Drawer 2: 0.1m from Drawer 1, prismatic x and z (w/ spring)


def drawer_viz():
    return f"""
<visual name="a">
  <geometry>
    <box>
      <size>0.5 0.25 0.02</size>
    </box>
  </geometry>
</visual>
""".strip()


def spring_base_viz():
    return f"""
<visual name="a">
  <geometry>
    <box>
      <size>0.02 0.02 0.02</size>
    </box>
  </geometry>
</visual>
""".strip()


def make_drawer(name, parent_frame, joint_type):
    start_frame = f"_{name}_start"
    spring_link = f"_{name}_spring_base"
    return f"""
    <frame name="{start_frame}">
      <pose relative_to="{parent_frame}">0 0 0.1  0 0 0</pose>
    </frame>

    <joint name="{name}_px" type="{joint_type}">
      <parent>base</parent>
      <child>{spring_link}</child>
      <axis>
        <xyz>1 0 0</xyz>
      </axis>
    </joint>

    <link name="{spring_link}">
      <pose relative_to="{start_frame}"/>
      {indent(spring_base_viz(), "      ").lstrip()}
    </link>

    <joint name="_{name}_pz_spring" type="{joint_type}">
      <parent>{spring_link}</parent>
      <child>{name}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-0.001</lower>
          <upper>0.001</upper>
        </limit>
      </axis>
    </joint>

    <link name="{name}">
      <pose relative_to="{start_frame}"/>
      {indent(drawer_viz(), "      ").lstrip()}
    </link>

""".strip()


def make_model(joint_type):
  return f"""
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="model_check">
    <link name="base"/>
    
    {make_drawer("drawer_1", "base", joint_type)}
    
    {make_drawer("drawer_2", "drawer_1", joint_type)}
  </model>
</sdf>
  """.lstrip()


def show_and_sim_model(model_file, q0=None):
    builder = DiagramBuilder()
    # time_step = 0.001  # good
    time_step = 0.1  # bad
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step)
    model = Parser(plant).AddModelFromFile(model_file)
    ConnectDrakeVisualizer(builder, scene_graph)
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
    plant.Finalize()
    diagram = builder.Build()

    diagram_context = diagram.CreateDefaultContext()
    simulator = Simulator(diagram, diagram_context)
    simulator.Initialize()
    context = plant.GetMyMutableContextFromRoot(diagram_context)
    num_v = plant.num_velocities()
    plant.get_actuation_input_port(model).FixValue(context, np.zeros(num_v))

    # This ordering is... so weird...
    for i in range(plant.num_joints()):
        joint = plant.get_joint(JointIndex(i))
        print(f"{i}: {joint.name()}, {joint.position_start()}")

    if q0 is not None:
      print("Set IC")
      plant.SetPositions(context, model, q0)
    print("Show kinematic")
    diagram.Publish(diagram_context)
    time.sleep(1)
    print("Show sim")
    simulator.set_target_realtime_rate(0.1)
    simulator.AdvanceTo(1.)


def show_joint_type(joint_type, q0=None):
    model_file = f"/tmp/model_check_{joint_type}.sdf"
    with open(model_file, "w") as f:
        f.write(make_model(joint_type))
    print(f"\n\n\nJoint Type: {joint_type}\n")
    show_and_sim_model(model_file, q0)
    print(" - Done")
    time.sleep(1)


assert __name__ == "__main__"

show_joint_type("prismatic", q0 = [
        # px: drawer 1 and 2
        0., 0.,
        # pz_spring: drawer 1 and 2
        0., 0.,
    ])
show_joint_type("fixed")
