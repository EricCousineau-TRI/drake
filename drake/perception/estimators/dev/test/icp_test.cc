#include "drake/perception/estimators/dev/icp.h"

#include <gtest/gtest.h>

#include "drake/math/roll_pitch_yaw.h"
#include "drake/perception/estimators/dev/test/test_util.h"

using Eigen::Matrix3Xd;
using Eigen::Vector3d;
using Eigen::Isometry3d;

namespace drake {
namespace perception {
namespace estimators {
namespace {

GTEST_TEST(Icp, PcaAndSvd) {
  // Use a bound whose principal axes (in order) are NOT (x, y, z), but
  // rather (z, x, y).
  const Bounds box(
      Interval(-0.05, 0.05),
      Interval(-0.01, 0.01),
      Interval(-0.2, 0.2));
  const double spacing = 0.01;
  // Generate points in body frame.
  Matrix3Xd points_B = GenerateBoxPointCloud(spacing, box);
  // Expect principal axes for PCA, where `Bp` is the body frame that aligns
  // with the principal axes.
  Isometry3d X_BB = Isometry3d::Identity();
  Isometry3d X_BBp = Isometry3d::Identity();
  X_BBp.linear() << Vector3d::UnitZ(), Vector3d::UnitX(), Vector3d::UnitY();
  // Show the points in the world frame with identity transform.
  // Note that `Bpi` is the body frame inferred by PCA in this instance.
  Isometry3d X_BBpi_pca = EstimatePcaBodyPose(points_B);
  const double tol = 1e-5;
  // Since our model is geometrically centered, this should be at or near
  // identity with PCA, with possible permutations on the axis signs.
  EXPECT_TRUE(CompareTransformWithoutAxisSign(X_BBp, X_BBpi_pca, tol));
  // Transform points to a pose in the world, such that the ground truth pose
  // is X_WB.
  Vector3d xyz(0.1, 0.2, 0.3);
  Vector3d rpy(kPi / 3, kPi / 11, kPi / 12);
  Isometry3d X_WB;
  X_WB.setIdentity();
  X_WB.linear() << drake::math::rpy2rotmat(rpy);
  X_WB.translation() << xyz;
  Matrix3Xd points_B_W = X_WB * points_B;
  // Quick meta-test on CompareTransformWithoutAxisSign.
  EXPECT_FALSE(CompareTransformWithoutAxisSign(X_BBp, X_WB, tol));
  // Compute PCA for body. Note that frame `Bj` is also inferred by PCA, and
  // may not represent the same frame as `Bi` due to axis sign permutations.
  Isometry3d X_WBpj_pca = EstimatePcaBodyPose(points_B_W);
  // Compute frame `Bk` aligned with intended body frame, rather than
  // principal axes. This will either be `B` if `Bpi` and `Bpj` are the
  // same, or will have axes aligned with `B`, but possibly with different
  // signs).
  Isometry3d X_WBk_pca = X_WBpj_pca * X_BBpi_pca.inverse();
  EXPECT_TRUE(CompareTransformWithoutAxisSign(X_WB, X_WBk_pca, tol));
  // Compute SVD for the body pose. Transformations should match, sign and all.
  Isometry3d X_WB_svd = ComputeSvdBodyPose(points_B, points_B_W);
  EXPECT_TRUE(CompareTransforms(X_WB, X_WB_svd, tol));
  // For visualization, use `show_frames`:
  //   drake-visualizer --script \
  //       ./drake/multibody/rigid_body_plant/visualization/show_frames.py
  SimpleVisualizer vis;
  vis.PublishCloud(points_B, "B");
  vis.PublishCloud(points_B_W, "BW");
  vis.PublishFrames({{"X_BB", X_BB},
                     {"X_BBp", X_BBp},
                     {"X_WB", X_WB},
                     {"X_BBpi_pca", X_BBpi_pca},
                     {"X_WBpj_pca", X_WBpj_pca},
                     {"X_WBk_pca", X_WBk_pca},
                     {"X_WB_svd", X_WB_svd}});
}

}  // namespace
}  // namespace estimators
}  // namespace perception
}  // namespace drake
