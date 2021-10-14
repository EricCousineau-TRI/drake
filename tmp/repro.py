assert __name__ == "__main__"

from pydrake.systems.framework import DiagramBuilder

from pydrake.geometry import DrakeVisualizer, SceneGraph
builder = DiagramBuilder()
scene_graph = SceneGraph()
DrakeVisualizer.AddToBuilder(builder, scene_graph)
