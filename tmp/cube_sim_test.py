import numpy as np
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant, AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.math import RigidTransform
from pydrake.geometry import HalfSpace, ProximityProperties


CUBE_DATA_POSITION_SLICE = slice(0,3,1)
CUBE_DATA_QUATERNION_SLICE = slice(3,7,1)
CUBE_DATA_VELOCITY_SLICE = slice(7,10,1)
CUBE_DATA_OMEGA_SLICE = slice(10,13,1)


default_drake_contact_params = {
    "mu": 0.18,
    "stiffness": 1.0e4, 
    "stiction_tol": 1e-3, 
    "dissipation":0.5 }

class DrakeCubeSim():

    def init_sim(self, params):
        ''' Here we build a diagram for the drake simulation'''
        self.builder = DiagramBuilder()
        
        # Add cube as MultibodyPlant
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, 1e-4)
        self.add_plant_and_terrain(params)

        self.diagram = self.builder.Build()
        self.diagram_context = self.diagram.CreateDefaultContext()
        
        self.sim = Simulator(self.diagram)
        self.sim.Initialize()

    def add_plant_and_terrain(self, params):
        # define ground 
        terrain_normal=np.array([0.0, 0.0, 1.0])
        terrain_point=np.zeros((3,))

        # contact parameters
        friction = CoulombFriction(params['mu'], params['mu'])
        props = ProximityProperties()
        props.AddProperty("material", "point_contact_stiffness", params['stiffness'])
        props.AddProperty("material", "hunt_crossley_dissipation", params['dissipation'])
        props.AddProperty("material", "coulomb_friction", friction)

        X_WG = RigidTransform(HalfSpace.MakePose(terrain_normal, terrain_point))

        ## Update model file location here ##
        Parser(self.plant).AddModelFromFile("tmp/cube.urdf")

        self.plant.RegisterCollisionGeometry(self.plant.world_body(), X_WG, HalfSpace(), "collision", props)
        self.plant.Finalize()
        self.plant.set_stiction_tolerance(params["stiction_tol"])

    def set_initial_condition(self, initial_state):

        q = np.zeros((self.plant.num_positions(),))
        v = np.zeros((self.plant.num_velocities(),))

        q[0:4] = initial_state[CUBE_DATA_QUATERNION_SLICE]
        q[4:] = initial_state[CUBE_DATA_POSITION_SLICE]
        v[0:3] = initial_state[CUBE_DATA_OMEGA_SLICE]
        v[3:] = initial_state[CUBE_DATA_VELOCITY_SLICE]

        self.sim.get_mutable_context().SetTime(0.0)
        self.sim.Initialize()

        self.plant.SetPositions(
            self.plant.GetMyMutableContextFromRoot(
                self.sim.get_mutable_context()), q)
        self.plant.SetVelocities(
            self.plant.GetMyMutableContextFromRoot(
                self.sim.get_mutable_context()), v)

    def simulate(self, time):
        self.sim.AdvanceTo(time)

if __name__ == '__main__':

    test_state = np.array([ 0.18629883,  0.02622872,  1.89283257, -0.52503014,  0.39360754,
       -0.29753734, -0.67794127,  0.01438053,  1.29095332, -0.21252927,
        1.46313532, -4.85439428,  9.86961928])

    sim = DrakeCubeSim()
    sim.init_sim(default_drake_contact_params)
    sim.set_initial_condition(test_state)
    sim.simulate(2.0)
