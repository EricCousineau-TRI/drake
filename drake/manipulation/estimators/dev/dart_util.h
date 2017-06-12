#pragma once

#include <memory>
#include <vector>
#include <map>

#include <fmt/format.h>

#include "drake/common/drake_throw.h"
#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/common/symbolic_variables.h"
#include "drake/solvers/decision_variable.h"

#include "drake/manipulation/estimators/dev/vector_slice.h"

namespace drake {
namespace manipulation {

using namespace std;
using namespace Eigen;

#define DRAKE_THROW_UNLESS_FMT(condition, ...) \
  if (!(condition)) { \
    ::drake::detail::Throw((std::string(#condition) + "\n" + \
        fmt::format(__VA_ARGS__)).c_str(), \
        __func__, __FILE__, __LINE__); \
  }

#define ASSERT_THROW_FMT(...) DRAKE_THROW_UNLESS_FMT(__VA_ARGS__)

typedef RigidBodyFrame<double> RigidBodyFramed;
typedef KinematicsCache<double> KinematicsCached;
typedef Matrix6X<double> Matrix6Xd;

typedef shared_ptr<const RigidBodyTreed> TreePtr;

typedef vector<int> Indices;
typedef map<string, int> InstanceIdMap;
typedef std::map<int, std::string> ReverseIdMap;

typedef solvers::VectorXDecisionVariable OptVars;

/**
 * Convenience to infer type.
 */
template <typename T>
std::unique_ptr<T> CreateUnique(T* obj) {
  return std::unique_ptr<T>(obj);
}


template <typename T, typename Container>
std::map<T, int> CreateIndexMap(const Container &x) {
  int i = 0;
  std::map<T, int> out;
  for (const auto& value : x) {
    DRAKE_ASSERT(out.find(value) == out.end());
    out[value] = i;
    i++;
  }
  return std::move(out);
}

/**
 * Simple mechanism to get the matching indices between two sets of lists.
 *
 * If a_indices and b_indices are supplied, then the indices returend for `a`
 * and `b` are returned in the order in which elements are found along `a`.
 *
 * If `b_indices` is null, then `a_indices` will be the same size as `a`,
 * and will store the correspondence between element `a[i]` and its position in
 * `b`.
 *
 * Note that this will not detect if all values are unique.
 */
// TODO(eric.cousineau): Split into two separate functions.
template <typename Container>
void GetCommonIndices(const Container &a,
                      const Container &b,
                      std::vector<int>* a_indices,
                      std::vector<int>* b_indices = nullptr,
                      bool verbose = false) {
  // TODO(eric.cousineau): See if there is a way to only store a reference.
  using T = std::remove_cv_t<std::decay_t<decltype(a[0])>>;
  auto a_map = CreateIndexMap<T>(a);
  auto b_map = CreateIndexMap<T>(b);
  vector<int> a_found(a.size(), false);
  vector<int> b_found(b.size(), false);
  for (const auto& a_pair : a_map) {
    auto b_iter = b_map.find(a_pair.first);
    if (b_iter != b_map.end()) {
      auto& b_pair = *b_iter;
      int& ai_found = a_found.at(a_pair.second);
      int& bi_found = b_found.at(b_pair.second);
      ASSERT_THROW_FMT(!ai_found && !bi_found,
                       "Duplicate elements:\n"
                       "a: {{ found: {}, value: {} }}\n"
                       "b: {{ found: {}, index: {} }}\n",
                       ai_found, a_pair.second,
                       bi_found, b_pair.second);
      if (b_indices) {
        a_indices->push_back(a_pair.second);
        b_indices->push_back(b_pair.second);
      } else {
        a_indices->push_back(b_pair.second);
      }
      ai_found = true;
      bi_found = true;
    } else {
      ASSERT_THROW_FMT(b_indices, "a[{}] not found in b, but a must be a subset"
                       " of b since b_indices was not provided.");
    }
  }
  if (verbose) {
    auto* log = drake::log();
    log->trace("  a not found:");
    for (int i = 0; i < (int)a.size(); ++i) {
      if (!a_found[i]) {
        log->trace("    {}", a[i]);
      }
    }
    log->trace("  b not found:");
    for (int i = 0; i < (int)b.size(); ++i) {
      if (!b_found[i]) {
        log->trace("    {}", b[i]);
      }
    }
  }
}


/**
 * Compute Cartesian jacobian of a point in the world, but attached to a body.
 * This will transform the point to the body frame of interest, and then compute
 * the relevant Jacobian.
 */
inline MatrixXd ComputeJacobianWithWorldPoints(
  const RigidBodyTreed& tree, const KinematicsCached& cache,
  const Vector3d& pt_W, const RigidBodyFramed* frame_A,
  bool in_terms_of_qdot) {
  // TODO(eric.cousineau): Check if this should require a body, not a frame,
  // to ensure additional constants don't sneak in.
  int frame_index = frame_A ? frame_A->get_frame_index() : 0;
  if (frame_index != 0) {
    Vector3d pt_A = tree.transformPoints(cache, pt_W, 0, frame_index);
    return tree.transformPointsJacobian(
      cache, pt_A, frame_index, 0, in_terms_of_qdot);
  } else {
    int nvar = in_terms_of_qdot ?
        tree.get_num_positions() : tree.get_num_velocities();
    return Matrix6Xd::Zero(6, nvar);
  }
}

/**
 * Compute the error in Cartesian position between two points in the world frame
 * `W`, and yield the affine (constant) and linear Jacobian of the error w.r.t.
 * the RigidBodyTree variables, for use in things such as a L2 error norm cost:
 *   e_lin(de) = e + Je*de
 * Frame A may be null, in which case it is assumed to be the world.
 *
 * This will perform the transformation in terms of q_dot for use in
 * optimization.
 */
inline void ComputeIcpPointToPointError(
  const RigidBodyTreed& tree, const KinematicsCached& cache,
  const RigidBodyFramed* frame_A, const Vector3d& ptA_W,
  const RigidBodyFramed* frame_B, const Vector3d& ptB_W,
  Vector3d* perror_W, MatrixXd* pJerror_W) {
  DRAKE_DEMAND(perror_W);
  DRAKE_DEMAND(pJerror_W);
  DRAKE_DEMAND(frame_B);

  auto& error_W = *perror_W;
  auto& Jerror_W = *pJerror_W;

  error_W = ptA_W - ptB_W;
  if (frame_A) {
    Jerror_W = ComputeJacobianWithWorldPoints(
        tree, cache, ptA_W, frame_A, true).topRows<3>();
  } else {
    Jerror_W.setZero();
  }
  Jerror_W -= ComputeJacobianWithWorldPoints(
        tree, cache, ptB_W, frame_B, true).topRows<3>();
}

// TODO(eric.cousineau): Merge functionality into WorldSimTreeBuilder.
// TODO(eric.cousineau): Find better location for this.

void PrintJointNameHierarchy(const RigidBodyTreed* tree);

void GetHierarchicalKinematicNameList(const RigidBodyTreed& tree, const ReverseIdMap& instance_name_map,
    vector<string>* position_names, vector<string>* velocity_names);

std::vector<std::string> GetHierarchicalPositionNameList(const RigidBodyTreed& tree,
    const ReverseIdMap& instance_name_map, bool add_velocity = false);

template <typename A, typename B>
map<B, A> ReverseMap(const map<A, B>& in) {
  map<B, A> out;
  for (const auto& pair : in) {
    out[pair.second] = pair.first;
  }
  return out;
}

inline void PrintValidPoints(const Eigen::Matrix3Xd& points,
                             const std::string& note) {
  int num_non_nan = 0;
  for (int i = 0; i < points.cols(); ++i) {
    if (!std::isnan(points(0, i))) {
      num_non_nan += 1;
    }
  }
  std::cout << fmt::format("Valid points: {} - {}\n", num_non_nan, note);
}

}  // manipulation
}  // drake
