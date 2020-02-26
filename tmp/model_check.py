
# coding: utf-8

# In[ ]:


from textwrap import indent
from pydrake.all import (
    DiagramBuilder, AddMultibodyPlantSceneGraph, Parser, Simulator, ConnectDrakeVisualizer,
    JointIndex,
)


# Model setup:
# * Base link
# * Drawer 1: 0.1m from bottom, prismatic x and z (w/ spring)
# * Drawer 2: 0.1m from Drawer 1, prismatic x and z (w/ spring)

# In[ ]:


drawer_viz = f"""
<visual name="a">
  <geometry>
    <box>
      <size>0.5 0.25 0.02</size>
    </box>
  </geometry>
</visual>
""".strip()

spring_base_viz = f"""
<visual name="a">
  <geometry>
    <box>
      <size>0.02 0.02 0.02</size>
    </box>
  </geometry>
</visual>
""".strip()

def make_drawer(name, parent_frame):
    start_frame = f"_{name}_start"
    spring_link = f"_{name}_spring_base"
    return f"""
    <frame name="{start_frame}">
      <pose relative_to="{parent_frame}">0 0 0.1  0 0 0</pose>
    </frame>

    <joint name="{name}_px" type="prismatic">
      <parent>base</parent>
      <child>{spring_link}</child>
      <axis>
        <xyz>1 0 0</xyz>
      </axis>
    </joint>

    <link name="{spring_link}">
      <pose relative_to="{start_frame}"/>
      {indent(spring_base_viz, "      ").lstrip()}
    </link>

    <joint name="_{name}_pz_spring" type="prismatic">
      <parent>{spring_link}</parent>
      <child>{name}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <dynamics>
          <spring_stiffness>1</spring_stiffness>
        </dynamics>
        <limit>
          <lower>0</lower>
          <upper>0</upper>
        </limit>
      </axis>
    </joint>

    <link name="{name}">
      <pose relative_to="{start_frame}"/>
      {indent(drawer_viz, "      ").lstrip()}
    </link>

""".strip()

model_text = f"""
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="model_check">
    <link name="base"/>
    
    {make_drawer("drawer_1", "base")}
    
    {make_drawer("drawer_2", "drawer_1")}
  </model>
</sdf>
""".lstrip()


# In[ ]:


model_file = "/tmp/model_check.sdf"
with open(model_file, "w") as f:
    f.write(model_text)


# In[ ]:


builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.1)
model = Parser(plant).AddModelFromFile(model_file)
ConnectDrakeVisualizer(builder, scene_graph)
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
plant.Finalize()
diagram = builder.Build()

Simulator(diagram).Initialize()
diagram_context = diagram.CreateDefaultContext()
context = plant.GetMyMutableContextFromRoot(diagram_context)

# This ordering is... so weird...
for i in range(plant.num_joints()):
    joint = plant.get_joint(JointIndex(i))
    print(f"{i}: {joint.name()}, {joint.position_start()}")

q = [
    # px: drawer 1 and 2
    0., 0.,
    # pz_spring: drawer 1 and 2
    0., 0.,
]
plant.SetPositions(context, model, q)
diagram.Publish(diagram_context)

