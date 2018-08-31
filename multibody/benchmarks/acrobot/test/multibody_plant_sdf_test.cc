#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant_sdf.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {

using Eigen::Vector3d;
using multibody::benchmarks::acrobot::AcrobotParameters;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using multibody::benchmarks::acrobot::MakeAcrobotPlantSdf;
using systems::Context;
using systems::LeafSystem;

namespace multibody {
namespace multibody_plant {
namespace {

// This test creates a simple model for an acrobot from an SDF file using
// MultibodyPlant and verifies a number of invariants such as that body and
// joint models were properly added and the model sizes.
GTEST_TEST(MultibodyPlant, SimpleModelCreationSdf) {
  const std::string kInvalidName = "InvalidName";
  std::unique_ptr<MultibodyPlant<double>> plant = MakeAcrobotPlantSdf();

  // MakePlantSdf() has already called Finalize() on the new acrobot
  // plant. Therefore attempting to call this method again will throw an
  // exception. Verify this.
  EXPECT_THROW(plant->Finalize(), std::logic_error);

  // Model Size. Counting the world body, there should be three bodies.
  EXPECT_EQ(3, plant->num_bodies());
  EXPECT_EQ(2, plant->num_joints());
  EXPECT_EQ(1, plant->num_actuators());
  EXPECT_EQ(1, plant->num_actuated_dofs());

  // State size.
  EXPECT_EQ(plant->num_positions(), 2);
  EXPECT_EQ(plant->num_velocities(), 2);
  EXPECT_EQ(plant->num_multibody_states(), 4);

  EXPECT_TRUE(plant->HasJointNamed("ShoulderJoint"));
  EXPECT_TRUE(plant->HasJointNamed("ElbowJoint"));
  EXPECT_FALSE(plant->HasJointNamed(kInvalidName));

  // Get links by name.
  const Body<double>& link1 = plant->GetBodyByName("Link1");
  EXPECT_EQ(link1.name(), "Link1");
  const Body<double>& link2 = plant->GetBodyByName("Link2");
  EXPECT_EQ(link2.name(), "Link2");

  // Get frames by name.
  const Frame<double>& link1_frame = plant->GetFrameByName("Link1");
  EXPECT_EQ(link1_frame.name(), "Link1");
  const Frame<double>& link2_frame = plant->GetFrameByName("Link2");
  EXPECT_EQ(link2_frame.name(), "Link2");
  // const Frame<double>& arbitrary_frame =
  //     plant->GetFrameByName("ArbitraryFrame");
  // EXPECT_EQ(arbitrary_frame.name(), "ArbitraryFrame");

  // Attempting to retrieve a link that is not part of the model should throw
  // an exception.
  EXPECT_THROW(plant->GetBodyByName(kInvalidName), std::logic_error);

  // Get joints by name.
  const Joint<double>& shoulder_joint = plant->GetJointByName("ShoulderJoint");
  EXPECT_EQ(shoulder_joint.name(), "ShoulderJoint");
  const Joint<double>& elbow_joint = plant->GetJointByName("ElbowJoint");
  EXPECT_EQ(elbow_joint.name(), "ElbowJoint");
  EXPECT_THROW(plant->GetJointByName(kInvalidName), std::logic_error);

  // Create a parameterized plant. This plant is generated
  // from Drake's API rather than from SDF.
  const AcrobotParameters parameters;
  std::unique_ptr<MultibodyPlant<double>> plant_parameterized =
      MakeAcrobotPlant(parameters, true /* Make a finalized plant. */);

  // Compute the parametertized model's mass matrix.
  std::unique_ptr<systems::Context<double>> context_parameterized =
    plant_parameterized->CreateDefaultContext();
  MatrixX<double> M_parameterized(2, 2);
  plant_parameterized->model().CalcMassMatrixViaInverseDynamics(
        *context_parameterized.get(), &M_parameterized);

  // Compute the SDF model's mass matrix.
  std::unique_ptr<systems::Context<double>> context =
    plant->CreateDefaultContext();
  MatrixX<double> M(2, 2);
  plant->model().CalcMassMatrixViaInverseDynamics(*context.get(), &M);

  // SDF's mass matrix should equal that generated by parameterized model.
  EXPECT_TRUE(CompareMatrices(M_parameterized, M));
}

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

