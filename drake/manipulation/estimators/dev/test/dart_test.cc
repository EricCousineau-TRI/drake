#include "drake/manipulation/estimators/dev/dart.h"

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/manipulation/estimators/dev/dart_util.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/sensors/rgbd_camera.h"

using namespace std;
using namespace drake::systems::sensors;

namespace drake {
namespace manipulation {
namespace {

// TODO(eric.cousineau): Consider mutable setup.
// TODO(eric.cousineau): Break into component-wise testing.
// TODO(eric.cousineau): Consider breaking MathematicalProgram into a
// Formulation and Solver components, such that it validates this and other
// designs. Make a DispatchSolver that can select from the variety at run-time,
// and can constraint on the solvers to choose.

typedef vector<pair<string, vector<string>>> InstanceJointList;
vector<string> FlattenNameList(const InstanceJointList& joints) {
  vector<string> joints_flat;
  for (const auto& pair : joints) {
    const auto& instance_name = pair.first;
    const auto& instance_joints = pair.second;
    for (const auto& joint_name : instance_joints) {
      joints_flat.push_back(instance_name + "::" + joint_name);
    }
  }
  return std::move(joints_flat);
}

class DartTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a formulation for a simple floating-base target
    InstanceIdMap instance_id_map;
    string file_path = GetDrakePath() +
        "/examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place.urdf";
    auto floating_base_type = multibody::joints::kRollPitchYaw;
    shared_ptr<RigidBodyFramed> weld_frame {nullptr};
    auto* mutable_tree = new RigidBodyTreed();
    instance_id_map["target"] =
        parsers::urdf::AddModelInstanceFromUrdfFile(
            file_path, floating_base_type,
            weld_frame, mutable_tree).begin()->second;
    mutable_tree->compile();

    // Add camera frame.
    const Vector3d position(2, 0, 0);
    const Vector3d orientation(0, 0, 0); // degrees
    const double pi = M_PI;

    auto* world_body = const_cast<RigidBody<double>*>(&mutable_tree->world());
    camera_frame_.reset(new RigidBodyFramed(
        "depth_sensor", world_body, position, orientation * pi / 180));
    mutable_tree->addFrame(camera_frame_);

    tree_.reset(mutable_tree);

    const double fov_y = pi / 4;
    rgbd_camera_sim_.reset(
        new RgbdCameraDirect(*tree_, *camera_frame_, fov_y, true));

    scene_ =
        new DartScene(tree_, instance_id_map);

    // Only interested in x-y-yaw (planar position) of the block
    DartFormulation::Param formulation_param {
      .estimated_positions = FlattenNameList({
          {"target", {"base_x", "base_y", "base_yaw"}} }),
    };
    formulation_ =
        new DartFormulation(CreateUnique(scene_), formulation_param);

    KinematicsState initial_state(scene_->tree());
    EXPECT_EQ(6, initial_state.q().size());
    EXPECT_EQ(6, initial_state.v().size());

//    DartJointObjective::Param joint_param = {
//      .joint_variance = {0.05, 0.05, 0.01},
//    };
//    joint_obj_ =
//        new DartJointObjective(formulation_, joint_param);

//    DartDepthImageIcpObjective::Param depth_param {
//      .camera_frame = camera_frame_,
//      .icp_variance = 0.05,
//      .free_space_variance = 0.005,
//      .downsample_factor = 5,
//      .point_cloud_bounds = {
//          .x = {-2, 2},
//          .y = {-2, 2},
//          .z = {-2, 2},
//      },
//    };
    depth_obj_ = DartDepthImageIcpObjective(formulation_, depth_param);

    // Tie things together.
    DartEstimator::Param estimator_param {
      .initial_state = initial_state,
    };
    estimator_.reset(
        new DartEstimator(CreateUnique(formulation_), estimator_param));
//    estimator_->AddObjective(CreateUnique(joint_obj_));
//    estimator_->AddObjective(CreateUnique(depth_obj_));
    estimator_->Compile();
  }

  void TearDown() override {}

  void SimulateDepthImage(double t, const VectorXd& q, ImageDepth32F* pdepth_image) {
    const int nq = tree_->get_num_positions();
    const int nv = tree_->get_num_velocities();
    VectorXd x(nq + nv);
    x.setZero();
    x.head(nq) = q;

    // Throw-away items.
    systems::rendering::PoseVector<double> pose;
    ImageBgra8U color_image;
    ImageLabel16I label_image;

    rgbd_camera_sim_->CalcImages(
        t, x,
        &pose, &color_image, pdepth_image, &label_image);
  }

  void Observe(double t, const KinematicsState& state_meas) {
    // Observe joint states.
    VectorXd v(tree_->get_num_velocities());
    v.setZero();
    joint_obj_->ObserveState(t, state_meas);

    // Simulate depth image.
    ImageDepth32F depth_image_meas;
    SimulateDepthImage(t, q_meas, &depth_image);
    depth_obj_->ObserveImage(t, depth_image_meas);
  }

  KinematicsState Update(double t) {
    // Update estimate.
    return estimator_->Update(t);
  }

 protected:
  shared_ptr<const RigidBodyTreed> tree_;
  shared_ptr<RigidBodyFramed> camera_frame_;
  DartScene* scene_;
  DartFormulation* formulation_;

  unique_ptr<RgbdCameraDirect> rgbd_camera_sim_;

  DartJointObjective* joint_obj_;
  DartDepthImageIcpObjective* depth_obj_;

  unique_ptr<DartEstimator> estimator_;
};

TEST_F(DartTest, BasicSetup) {
  double t = 0;
  KinematicsState state_meas(*tree_);
  estimator_->Observe(t, state_meas);
  KinematicsState state_est = estimator_->Update(t);
}

}  // namespace
}  // namespace manipulation
}  // namespace drake
