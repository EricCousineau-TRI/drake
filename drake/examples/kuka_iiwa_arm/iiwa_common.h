#pragma once

#include <memory>
#include <string>
#include <vector>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

constexpr int kIiwaArmNumJoints = 7;

/// Computes the lumped inertia parameters of the gripper and the end effector
/// link expressed in the end effector frame.
/// @param world_tree The RigidBodyTree that contains the arm and the gripper
/// models.
/// @param iiwa_instance Identifier for the arm in @p world_tree.
/// @param end_effector_link_name Link name of the end effector.
/// @param wsg_instance Identifier for the gripper in @p world_tree.
/// @param ee_fixed_id Identifiers for additional instances that
/// are fixed to the end effector.
/// @return Lumped inertia parameters.
template <typename T>
Matrix6<T> ComputeLumpedGripperInertiaInEndEffectorFrame(
    const RigidBodyTree<T>& world_tree,
    int iiwa_instance, const std::string& end_effector_link_name,
    int wsg_instance, const std::vector<int>& ee_fixed_id = {});

/// Verifies that @p tree matches assumptions about joint indices.
/// Aborts if the tree isn't as expected.
void VerifyIiwaTree(const RigidBodyTree<double>& tree);

/// Builds a RigidBodyTree at the specified @position and @orientation from
/// the model specified by @model_file_name.
/// This method is a convinience wrapper over `AddModelInstanceFromUrdfFile`.
/// @see drake::parsers::urdf::AddModelInstanceFromUrdfFile
void CreateTreedFromFixedModelAtPose(
    const std::string& model_file_name, RigidBodyTreed* tree,
    const Eigen::Vector3d& position = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& orientation = Eigen::Vector3d::Zero());

/// Used to set the feedback gains for the simulated position controlled KUKA.
void SetPositionControlledIiwaGains(Eigen::VectorXd* Kp,
                                    Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd);

/// Scales a plan so that no step exceeds the maximum joint velocity
/// specified.  The number of columns in @p keyframes must match the
/// size of @p time.  Times must be in strictly increasing order.
void ApplyJointVelocityLimits(double max_joint_velocity,
                              const MatrixX<double>& keyframes,
                              std::vector<double>* time);


/// Makes a robotlocomotion::robot_plan_t message.  The number of
/// columns in @p keyframes must match the size of @p time.  Times
/// must be in strictly increasing order.
robotlocomotion::robot_plan_t EncodeKeyFrames(
    const RigidBodyTree<double>& robot, const std::vector<double>& time,
    const std::vector<int>& info, const MatrixX<double>& keyframes);

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
