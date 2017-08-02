#pragma once

#include <bot_core/pointcloud_t.hpp>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmtypes/drake/lcmt_viewer_draw.hpp"

namespace drake {
namespace perception {
namespace estimators {

const double kPi = M_PI;

// TODO(eric.cousineau): Move to a proper LCM conversion type.
void PointCloudToLcm(const Eigen::Matrix3Xd& pts_W,
                     bot_core::pointcloud_t* pmessage);

/*
 * Simple interval class.
 */
struct Interval {
  Interval() {}
  Interval(double min_in, double max_in)
      : min(min_in), max(max_in) {
    DRAKE_DEMAND(min <= max);
  }
  double min{};
  double max{};
  inline bool IsInside(double i) const { return i >= min && i <= max; }
  inline double width() const { return max - min; }
};

struct Bounds {
  Bounds() {}
  Bounds(Interval x_in, Interval y_in, Interval z_in)
      : x(x_in), y(y_in), z(z_in) {}
  Interval x;
  Interval y;
  Interval z;
  inline bool IsInside(double xi, double yi, double zi) const {
    return x.IsInside(xi) && y.IsInside(yi) && z.IsInside(zi);
  }
};

struct IntervalIndex {
  int index;
  Interval interval;
};

struct PlaneIndices {
  IntervalIndex a;  // first plane coordinate
  IntervalIndex b;  // second plane coordinate
  IntervalIndex d;  // depth plane coordinate
};

Eigen::Matrix2Xd Generate2DPlane(double space, Interval x, Interval y);

Eigen::Matrix3Xd Generate2DPlane(double space, PlaneIndices is);

Eigen::Matrix3Xd GenerateBoxPointCloud(double space, Bounds box);

/*
 * Compare two transforms as matrices.
 */
::testing::AssertionResult CompareTransforms(const Eigen::Isometry3d& A,
                                             const Eigen::Isometry3d& B,
                                             double tolerance = 0.0);

/*
 * Compare `R_actual` against `R_expected` to ensure that the axes are
 * aligned, but may have different signs. This is done by checking:
 *   tr(abs(R_expected' R_actual)) == 3
 */
::testing::AssertionResult CompareRotationWithoutAxisSign(
    const Eigen::Matrix3d& R_expected, const Eigen::Matrix3d& R_actual,
    double tolerance = 0.0);

/*
 * Compare `X_actual` against `X_expected`, first comparing translation, and
 * then comparing rotations using CompareRotationWithoutAxisSign.
 */
::testing::AssertionResult CompareTransformWithoutAxisSign(
    const Eigen::Isometry3d& X_expected, const Eigen::Isometry3d& X_actual,
    double tolerance = 0.0);

class IcpVisualizer {
 public:
  void PublishCloud(const Eigen::Matrix3Xd& points,
                    const std::string& suffix = "RGBD");

  void PublishFrames(
      const std::vector<std::pair<std::string, Eigen::Isometry3d>>& frames);

  lcm::DrakeLcm& lcm() { return lcm_; }
 private:
  lcm::DrakeLcm lcm_;
};

}  // namespace estimators
}  // namespace perception
}  // namespace drake
