
import unittest
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.eigen_geometry import Isometry3
from pydrake.examples.manipulation_station import (
    IiwaCollisionModel,
    ManipulationStation,
    ManipulationStationHardwareInterface
)
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.multibody_tree.parsing import Parser
from pydrake.multibody.multibody_tree import ModelInstanceIndex


class TestManipulationStation(unittest.TestCase):
    def test_manipulation_station(self):
        # Just check the spelling.
        station = ManipulationStation(time_step=0.001)
        station.SetupDefaultStation()
        station.SetWsgGains(0.1, 0.1)
        station.SetIiwaPositionGains(np.ones(7))
        station.SetIiwaVelocityGains(np.ones(7))
        station.SetIiwaIntegralGains(np.ones(7))
        station.Finalize()
        station.get_multibody_plant()
        station.get_mutable_multibody_plant()
        station.get_scene_graph()
        station.get_mutable_scene_graph()
        station.get_controller_plant()

        # Check the setters/getters.
        context = station.CreateDefaultContext()
        q = np.linspace(0.04, 0.6, num=7)
        v = np.linspace(-2.3, 0.5, num=7)
        station.SetIiwaPosition(q, context)
        np.testing.assert_array_equal(q, station.GetIiwaPosition(context))
        station.SetIiwaVelocity(v, context)
        np.testing.assert_array_equal(v, station.GetIiwaVelocity(context))

        q = 4.23
        v = 8.51
        station.SetWsgPosition(q, context)
        self.assertEqual(q, station.GetWsgPosition(context))
        station.SetWsgVelocity(v, context)
        self.assertEqual(v, station.GetWsgVelocity(context))

        station.get_camera_poses_in_world()["0"]
        self.assertEqual(len(station.get_camera_names()), 3)

    def test_manipulation_station_add_iiwa_and_wsg_explicitly(self):
        station = ManipulationStation()
        parser = Parser(station.get_mutable_multibody_plant(),
                        station.get_mutable_scene_graph())
        plant = station.get_mutable_multibody_plant()

        # Add models for iiwa and wsg
        iiwa_model_file = FindResourceOrThrow(
            "drake/manipulation/models/iiwa_description/iiwa7/"
            "iiwa7_no_collision.sdf")
        iiwa = parser.AddModelFromFile(iiwa_model_file, "iiwa")
        X_WI = Isometry3.Identity()
        plant.WeldFrames(plant.world_frame(),
                         plant.GetFrameByName("iiwa_link_0", iiwa), X_WI)

        wsg_model_file = FindResourceOrThrow(
            "drake/manipulation/models/wsg_50_description/sdf/"
            "schunk_wsg_50.sdf")
        wsg = parser.AddModelFromFile(wsg_model_file, "gripper")
        X_7G = Isometry3.Identity()
        plant.WeldFrames(
            plant.GetFrameByName("iiwa_link_7", iiwa),
            plant.GetFrameByName("body", wsg), X_7G)

        # Register models for the controller.
        station.RegisterIiwaControllerModel(
            iiwa_model_file, iiwa, plant.world_frame(),
            plant.GetFrameByName("iiwa_link_0", iiwa), X_WI)
        station.RegisterWsgControllerModel(
            wsg_model_file, wsg,
            plant.GetFrameByName("iiwa_link_7", iiwa),
            plant.GetFrameByName("body", wsg), X_7G)

        # Finalize
        station.Finalize()

        # This WSG gripper model has 2 independent dof, and the IIWA model
        # has 7.
        self.assertEqual(plant.num_positions(), 9)
        self.assertEqual(plant.num_velocities(), 9)

    def test_iiwa_collision_model(self):
        # Check that all of the elements of the enum were spelled correctly.
        IiwaCollisionModel.kNoCollision
        IiwaCollisionModel.kBoxCollision

    def test_manipulation_station_hardware_interface(self):
        station = ManipulationStationHardwareInterface(
            camera_names=["123", "456"])
        # Don't actually call Connect here, since it would block.
        station.get_controller_plant()
        self.assertEqual(len(station.get_camera_names()), 2)
