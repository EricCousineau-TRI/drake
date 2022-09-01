#include <gtest/gtest.h>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"

namespace drake {
namespace multibody {
namespace {

// Creates plant with 7 DoFs:
// - 1 DoFs belongs to default model instance
// - 1 DoFs belongs to main model instance
// - 5 DoFs belongs to another model; however, the outboard bodies are
//   influenced by the main model instance.
std::pair<std::unique_ptr<MultibodyPlant<double>>, ModelInstanceIndex>
MakeDummyPlant() {
  const double time_step = 0.0;
  auto plant = std::make_unique<MultibodyPlant<double>>(time_step);
  const Eigen::Vector3d dummy_axis(1.0, 0.0, 0.0);

  const ModelInstanceIndex model_instance = plant->AddModelInstance("my_model");
  const ModelInstanceIndex other_instance =
      plant->AddModelInstance("other_model");

  auto add_joint_and_body = [&](
      ModelInstanceIndex model,
      std::string parent_body_name,
      std::string joint_name,
      std::string child_body_name) {
    const Body<double>& parent = plant->GetBodyByName(parent_body_name);
    const Body<double>& child = plant->AddRigidBody(
      child_body_name, model, SpatialInertia<double>());
    const Joint<double>& joint = plant->AddJoint<RevoluteJoint>(
        joint_name, parent, {}, child, {}, dummy_axis);
    EXPECT_EQ(joint.model_instance(), model);
  };

  // Create joint_a and body_a which are part of the default model instance
  // (which we don't really care about).
  add_joint_and_body(
      default_model_instance(), "WorldBody", "joint_a", "body_a");
  // Create joint_b and body_b which are part of our custom model instance.
  add_joint_and_body(model_instance, "body_a", "joint_b", "body_b");
  // Create additional joints which are not part of our custom model instance,
  // but whose kinematics are influenced by joint_b.
  add_joint_and_body(other_instance, "body_b", "joint_c", "body_c");
  // - Serial.
  add_joint_and_body(other_instance, "body_c", "joint_d", "body_d");
  // - Branch from body_b.
  add_joint_and_body(other_instance, "body_b", "joint_e", "body_e");
  add_joint_and_body(other_instance, "body_b", "joint_f", "body_f");
  // - Serial on branch from body_e.
  add_joint_and_body(other_instance, "body_e", "joint_g", "body_g");
  // Do some minor checks.
  plant->Finalize();
  EXPECT_EQ(plant->num_positions(), 7);
  EXPECT_EQ(plant->num_positions(model_instance), 1);
  return std::make_pair(std::move(plant), model_instance);
}

GTEST_TEST(
    DofTest, ComputeBodiesKinematicallyInfluencedByActiveDof) {
  auto [plant, model_instance] = MakeDummyPlant();
  const std::vector<JointIndex> joints = plant->GetJointIndices(model_instance);
  const std::vector<BodyIndex> bodies = plant->GetBodiesAffectedBy(joints);
  auto body_index = [&](std::string body_name) {
    return plant->GetBodyByName(body_name).index();
  };
  const std::vector<BodyIndex> bodies_expected{
      body_index("body_b"),
      body_index("body_c"),
      body_index("body_d"),
      body_index("body_e"),
      body_index("body_f"),
      body_index("body_g")};
  EXPECT_EQ(bodies, bodies_expected);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
