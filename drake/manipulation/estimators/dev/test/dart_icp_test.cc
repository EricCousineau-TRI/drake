#include "drake/manipulation/estimators/dev/dart.h"
#include "drake/manipulation/estimators/dev/dart_objectives.h"
#include "drake/manipulation/estimators/dev/dart_depth_image_icp_objective.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/manipulation/estimators/dev/dart_util.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmtypes/drake/lcmt_viewer_draw.hpp"
#include "drake/lcmtypes/drake/lcmt_viewer_load_robot.hpp"
#include "drake/multibody/rigid_body_plant/create_load_robot_message.h"
#include "bot_core/pointcloud_t.hpp"
#include "external/lcmtypes_bot2_core/lcmtypes/bot_core/pointcloud_t.hpp"

using namespace std;
using namespace drake::systems::sensors;

namespace drake {
namespace manipulation {
namespace {

// TODO(eric.cousineau): Consider processing with VTK.
// TODO(eric.cousineau): Consider breaking MathematicalProgram into a
// Formulation and Solver components, such that it validates this and other
// designs. Make a DispatchSolver that can select from the variety at run-time,
// and can constraint on the solvers to choose.

struct IntervalIndex {
  int index;
  Interval interval;
};

struct PlaneIndices {
  IntervalIndex a;  // first plane coordinate
  IntervalIndex b;  // second plane coordinate
  IntervalIndex d;  // depth plane coordinate
};

Matrix2Xd GeneratePlane(double space, Interval x, Interval y) {
  const int nc = floor(x.width() / space);
  const int nr = floor(y.width() / space);
  int i = 0;
  Matrix2Xd out(2, nc * nr);
  for (int c = 0; c < nc; c++) {
    for (int r = 0; r < nr; r++) {
      out.col(i) << c * space + x.min, r * space + y.min;
      i++;
    }
  }
  out.conservativeResize(Eigen::NoChange, i);
  return out;
}

Matrix3Xd GeneratePlane(double space, PlaneIndices is) {
  // Generate single plane.
  Matrix2Xd p2d = GeneratePlane(space, is.a.interval, is.b.interval);
  // Map to 3d for upper and lower bound
  const int n = p2d.cols();
  Matrix3Xd p3du(3, 2 * n);
  auto map_into = [&](auto&& xpr, double value) {
    xpr.row(is.a.index) = p2d.row(0);
    xpr.row(is.b.index) = p2d.row(1);
    xpr.row(is.d.index).setConstant(value);
  };
  map_into(p3du.leftCols(n), is.d.interval.min);
  map_into(p3du.rightCols(n), is.d.interval.max);
  return p3du;
}

Matrix3Xd GenerateBoxPointCloud(double space, Bounds box) {
  IntervalIndex ix = {0, box.x};
  IntervalIndex iy = {1, box.y};
  IntervalIndex iz = {2, box.z};
  // Generate for each face
  auto xy_z = GeneratePlane(space, {ix, iy, iz});
  auto yz_x = GeneratePlane(space, {iy, iz, ix});
  auto xz_y = GeneratePlane(space, {ix, iz, iy});
  Matrix3Xd pts(3, xy_z.cols() + yz_x.cols() + xz_y.cols());
  pts << xy_z, yz_x, xz_y;
  return pts;
}

struct PointCorrespondence {
  // TODO(eric.cousineau): Consider storing normals, different frames, etc.
  /** @brief Measured feature index (point). */
  int meas_index{};
  /** @brief Measured point. */
  Vector3d meas_point;
  /** @brief Model feature index (body). */
  int model_index{};
  /** @brief Model point, same frame as measured point. */
  Vector3d model_point;
  /** @brief Distance between the points. */
  double distance{-1};
};

typedef map<BodyIndex, vector<PointCorrespondence>> SceneCorrespondences;

struct BodyCorrespondenceInfluence {
  /**
   * @brief Will a correspondence with this body affect the camera
   * positions?
   */
  bool will_affect_camera{};
  /**
   * @brief Will a correspondence with this body affect the body's kinematics?
   */
  bool will_affect_body{};
  /**
   * @brief Check if a correspondence with this body will influence the
   * Jacobian at all.
   */
  bool is_influential() const {
    return will_affect_camera || will_affect_body;
  }
};
typedef vector<BodyCorrespondenceInfluence> Influences;

void GetDofPath(const RigidBodyTreed& tree,
                BodyIndex body_index,
                std::vector<int> *pq_indices) {
  // NOTE: FindKinematicPath will only return joint information, but not DOF
  // information.
  auto& q_indices = *pq_indices;
  const int frame_W = 0;

  KinematicPath kinematic_path =
      tree.findKinematicPath(frame_W, body_index);
  int q_index = 0;
  // Adapted from: RigidBodyTree<>::geometricJacobian
  for (int cur_body_index : kinematic_path.joint_path) {
    const DrakeJoint& joint = tree.get_body(cur_body_index).getJoint();
    for (int i = 0; i < joint.get_num_positions(); ++i) {
      q_indices.push_back(q_index++);
    }
  }
}

template <typename ContainerA, typename ContainerB>
bool HasIntersection(const ContainerA& a, const ContainerB& b) {
  for (const auto& ai : a) {
    if (std::find(b.begin(), b.end(), ai) != b.end()) {
      return true;
    }
  }
  return false;
}

/**
 * Will a correspondence with a given body influence the optimization
 * formulation (camera or body position)? Will it mathematically affect
 * anything, or should computations relevant to this be skipped?
 * @param scene
 * @param ComputeCorrespondences
 */
BodyCorrespondenceInfluence IsBodyCorrespondenceInfluential(
    const IcpScene& scene,
    BodyIndex body_index,
    const VectorSlice *q_slice) {
  BodyCorrespondenceInfluence out;
  out.will_affect_camera = (scene.frame_C != scene.frame_W);
  if (q_slice) {
    // Check if this body's kinematic chain will be affected by the slice.
    vector<int> q_indices;
    GetDofPath(scene.tree, body_index, &q_indices);
    // If there is any intersection, then this will influence things.
    out.will_affect_body =
        HasIntersection(q_indices, q_slice->indices());
  } else {
    out.will_affect_body = true;
  }
  return out;
}

void ComputeBodyCorrespondenceInfluences(
    const IcpScene& scene,
    const VectorSlice* q_slice,
    Influences* influences) {
  int num_bodies = scene.tree.get_num_bodies();
  influences->resize(num_bodies);
  for (int i = 0; i < num_bodies; ++i) {
    (*influences)[i] = IsBodyCorrespondenceInfluential(scene, i, q_slice);
  }
}

/**
 * This function depends on `q`.
 * @param scene
 * @param meas_pts_W
 * @param pcorrespondence
 */
void ComputeCorrespondences(const IcpScene& scene,
                            const Matrix3Xd& meas_pts_W,
                            const Influences& influences,
                            SceneCorrespondences* pcorrespondence) {
  DRAKE_DEMAND(pcorrespondence != nullptr);
  int num_points = meas_pts_W.cols();
  VectorXd distances(num_points);
  Matrix3Xd body_normals_W(3, num_points);  // world frame.
  Matrix3Xd body_pts_W(3, num_points);  // body point, world frame.
  Matrix3Xd body_pts_Bi(3, num_points);  // body point, body frame.
  vector<BodyIndex> body_indices(num_points);

  const bool use_margins = false;
  // TOOD(eric.cousineau): Figure out better access.
  RigidBodyTreed& mutable_tree =
      const_cast<RigidBodyTreed&>(scene.tree);
  mutable_tree.collisionDetectFromPoints(
      scene.cache, meas_pts_W,
      distances, body_normals_W,
      body_pts_W, body_pts_Bi,
      body_indices, use_margins);

  // Cache whether or not there is influence. This should be effectively used
  // to filter out points from the set.
  int num_bodies = scene.tree.get_num_bodies();
  vector<bool> is_body_influential(num_bodies);
  for (int i = 0; i < num_bodies; ++i) {
    is_body_influential[i] = influences[i].is_influential();
  }

  // Bin each correspondence to the given body.
  for (int i = 0; i < num_points; ++i) {
    BodyIndex body_index = body_indices[i];
    if (body_index != -1 && is_body_influential[body_index]) {
      vector<PointCorrespondence>& body_correspondences =
          (*pcorrespondence)[body_index];
      PointCorrespondence pc{
          i, meas_pts_W.col(i), body_index, body_pts_W.col(i), distances[i]};
      body_correspondences.push_back(pc);
    }
  }
}

typedef std::function<void(const IcpPointGroup&)> CostAggregator;

void AggregateCost(const IcpScene& scene,
                     const SceneCorrespondences& correspondence,
                     const CostAggregator& aggregate) {
  for (const auto& pair : correspondence) {
    const BodyIndex body_index = pair.first;
    const vector<PointCorrespondence>& body_correspondence = pair.second;
    IcpPointGroup body_pts_group(body_index, body_correspondence.size());
    for (const auto& pc_W : body_correspondence) {
      body_pts_group.Add(pc_W.meas_point, pc_W.model_point);
    }
    body_pts_group.Finalize();
    aggregate(body_pts_group);
  }
}

class ConstantCostAggregator {
 public:
  ConstantCostAggregator(const IcpScene& scene)
    : scene_(&scene), cost_(0) {}

  void operator()(const IcpPointGroup& pts) {
    pts.UpdateError(*scene_, &es, &Jes);
    // Get error squared.
    cost_ += (es.transpose() * es).sum();
  }
  double cost() const {
    return cost_;
  }
 private:
  const IcpScene* scene_;
  Matrix3Xd es;
  MatrixXd Jes;
  double cost_{};
};

class QPCostAggregator {
 public:
  QPCostAggregator(const IcpScene& scene, double weight)
      : error_accumulator(scene.tree.get_num_positions()),
        scene(&scene), weight(weight) {}

  void operator()(const IcpPointGroup& body_pts_group) {
    body_pts_group.UpdateError(*scene, &es, &Jes);
    error_accumulator.AddTerms(weight, es, Jes);
  }

  IcpLinearizedNormAccumulator error_accumulator;
 private:
  const IcpScene* scene;
  double weight;
  Matrix3Xd es;
  MatrixXd Jes;
};

#if false
void AccumulateQPCost(const IcpScene& scene,
                      const SceneCorrespondences& correspondence,
                      double weight,
                      const VectorSlice& slice,
                      QuadraticCost* cost) {
  QPCostAggregator aggregator(scene, weight);
  AggregateCost(scene, correspondence, std::ref(aggregator));
  aggregator.error_accumulator.UpdateCost(slice, cost);
}
#endif

class DartIcpTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a formulation for a simple floating-base target
    string file_path = FindResourceOrThrow(
        "drake/manipulation/estimators/dev/simple_cuboid.urdf");
    auto floating_base_type = multibody::joints::kRollPitchYaw;
    shared_ptr<RigidBodyFramed> weld_frame {nullptr};
    mutable_tree_ = new RigidBodyTreed();
    parsers::urdf::AddModelInstanceFromUrdfFile(
        file_path, floating_base_type,
        weld_frame, mutable_tree_);
    // TODO(eric.cousineau): Figure out why ground is absorbing points...
    drake::multibody::AddFlatTerrainToWorld(mutable_tree_);
    mutable_tree_->compile();
    tree_.reset(mutable_tree_);
    tree_cache_.reset(new KinematicsCached(tree_->CreateKinematicsCache()));

    Vector3d obj_pos(0, 0, 0.25);
    q0_.resize(6);
    q0_.head(3) << obj_pos;

//    DartFormulation::Param formulation_param {
//      .estimated_positions = FlattenNameList({
//          {"target", {"base_x", "base_y", "base_yaw"}} }),
//    };
    q_slice_.reset(new VectorSlice({0, 1, 5}, 6));

    // Add camera frame.
    Vector3d position(-2, 0, 0.1);
    position += obj_pos;
    const Vector3d orientation(0, 0, 0); // degrees
    const double pi = M_PI;

    auto* world_body = const_cast<RigidBody<double>*>(&tree_->world());
    shared_ptr<RigidBodyFramed> camera_frame;
    camera_frame.reset(new RigidBodyFramed(
        "depth_sensor", world_body, position, orientation * pi / 180));
    mutable_tree_->addFrame(camera_frame);

    scene_.reset(new IcpScene(*tree_, *tree_cache_, 0,
                              camera_frame->get_frame_index()));

    ComputeBodyCorrespondenceInfluences(*scene_, q_slice_.get(),
                                        &influences_);

    const Bounds box = {
      .x = {-0.03, 0.03},
      .y = {-0.03, 0.03},
      .z = {-0.1, 0.1},
    };
    const double space = 0.02;
    points_ = GenerateBoxPointCloud(space, box);
    points_.colwise() += obj_pos;
  }

 protected:
  RigidBodyTreed* mutable_tree_;
  unique_ptr<KinematicsCached> tree_cache_;
  shared_ptr<const RigidBodyTreed> tree_;
  VectorXd q0_;
  unique_ptr<IcpScene> scene_;
  unique_ptr<VectorSlice> q_slice_;
  Influences influences_;
  Matrix3Xd points_;
};

typedef Matrix3Xd Data;
typedef bot_core::pointcloud_t Message;
void PointCloudToLcm(const Matrix3Xd& pts_W, Message* pmessage) {
  Message& message = *pmessage;
  message.points.clear();
  message.frame_id = std::string(RigidBodyTreeConstants::kWorldName);
  message.n_channels = 0;
  message.n_points = pts_W.cols();
  message.points.resize(message.n_points);
  for (int i = 0; i < message.n_points; ++i) {
    Eigen::Vector3f pt_W = pts_W.col(i).cast<float>();
    message.points[i] = {pt_W(0), pt_W(1), pt_W(2)};
  }
  message.n_points = message.points.size();
}

void Visualize(const IcpScene& scene, const Matrix3Xd& points,
               SceneCorrespondences& correspondence) {
  lcm::DrakeLcm lcmc;
  using namespace systems;
  {
    const lcmt_viewer_load_robot load_msg(
        (multibody::CreateLoadRobotMessage<double>(scene.tree)));
    vector<uint8_t> bytes(load_msg.getEncodedSize());
    load_msg.encode(bytes.data(), 0, bytes.size());
    lcmc.Publish("DRAKE_VIEWER_LOAD_ROBOT", bytes.data(), bytes.size());
  }
  {
    const ViewerDrawTranslator draw_msg_tx(scene.tree);
    drake::lcmt_viewer_draw draw_msg;
    vector<uint8_t> bytes;
    const int nq = scene.tree.get_num_positions();
    const int nv = scene.tree.get_num_velocities();
    const int nx = nq + nv;
    BasicVector<double> x(nx);
    auto xb = x.get_mutable_value();
    xb.setZero();
    xb.head(nq) = scene.cache.getQ();
    draw_msg_tx.Serialize(0, x, &bytes);
    lcmc.Publish("DRAKE_VIEWER_DRAW", bytes.data(), bytes.size());
  }
  // Publish point cloud
  {
    bot_core::pointcloud_t pt_msg;
    PointCloudToLcm(points, &pt_msg);
    vector<uint8_t> bytes(pt_msg.getEncodedSize());
    pt_msg.encode(bytes.data(), 0, bytes.size());
    lcmc.Publish("DRAKE_POINTCLOUD_RGBD", bytes.data(), bytes.size());
  }
}

TEST_F(DartIcpTest, PositiveReturnsZeroCost) {
  // Start box at the given state, ensure that the cost returned is near zero.
  VectorXd q0 = q0_;
  tree_cache_->initialize(q0);
  tree_->doKinematics(*tree_cache_);

  // Get correspondences
  SceneCorrespondences correspondence;
  ComputeCorrespondences(*scene_, points_, influences_, &correspondence);

  Visualize(*scene_, points_, correspondence);

  // Compute error
  ConstantCostAggregator aggregator(*scene_);
  AggregateCost(*scene_, correspondence, std::ref(aggregator));

  double tol = 1e-10;
  // The error should be near zero for points on the surface.
  EXPECT_NEAR(0, aggregator.cost(), tol);
}

}  // namespace
}  // namespace manipulation
}  // namespace drake
