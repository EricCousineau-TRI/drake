#include "drake/perception/point_cloud.h"

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/drake_assert.h"

using Eigen::Map;
using Eigen::NoChange;

namespace drake {
namespace perception {

namespace {

// Convenience aliases.
typedef PointCloud::T T;
typedef PointCloud::D D;

}  // namespace

/*
 * Provides encapsulated storage for a `PointCloud`.
 *
 * This storage is not responsible for initializing default values.
 */
class PointCloud::Storage {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Storage)

  Storage(int new_size, pc_flags::Fields fields)
      : fields_(fields) {
    // Ensure that we incorporate the size of the descriptors.
    descriptors_.resize(fields_.descriptor_type().size(), 0);
    // Resize as normal.
    resize(new_size);
  }

  // Returns size of the storage.
  int size() const { return size_; }

  // Resize to parent cloud's size.
  void resize(int new_size) {
    size_ = new_size;
    if (fields_.has(pc_flags::kXYZs))
      xyzs_.conservativeResize(NoChange, new_size);
    if (fields_.has_descriptor())
      descriptors_.conservativeResize(NoChange, new_size);
    CheckInvariants();
  }

  Eigen::Ref<Matrix3X<T>> xyzs() { return xyzs_; }
  Eigen::Ref<MatrixX<T>> descriptors() { return descriptors_; }

 private:
  void CheckInvariants() const {
    if (fields_.has(pc_flags::kXYZs)) {
      const int xyz_size = xyzs_.cols();
      DRAKE_DEMAND(xyz_size == size());
    }
    if (fields_.has_descriptor()) {
      const int descriptor_size = descriptors_.cols();
      DRAKE_DEMAND(descriptor_size == size());
    }
  }

  const pc_flags::Fields fields_;
  int size_{};
  Matrix3X<T> xyzs_;
  MatrixX<T> descriptors_;
};

namespace {

pc_flags::Fields ResolveFields(
    const PointCloud& other, pc_flags::Fields fields) {
  if (fields == pc_flags::kInherit) {
    return other.fields();
  } else {
    return fields;
  }
}

// Implements the rules set forth in `SetFrom`.
// @pre Valid point clouds `a` and `b`.
// @post The returned fields will be valid for both point clouds.
pc_flags::Fields ResolvePointCloudPairFields(
    const PointCloud& a,
    const PointCloud& b,
    pc_flags::Fields fields) {
  if (fields == pc_flags::kInherit) {
    // If we do not permit a subset, expect the exact same fields.
    a.RequireExactFields(b.fields());
    return a.fields();
  } else {
    a.RequireFields(fields);
    b.RequireFields(fields);
    return fields;
  }
}

}  // namespace

PointCloud::PointCloud(
    int new_size,
    pc_flags::Fields fields)
    : size_(new_size),
      fields_(fields) {
  if (fields_ == pc_flags::kNone)
    throw std::runtime_error("Cannot construct a PointCloud without fields");
  if (fields_.has(pc_flags::kInherit))
    throw std::runtime_error("Cannot construct a PointCloud with kInherit");
  storage_.reset(new Storage(size_, fields_));
  SetDefault(0, size_);
}

PointCloud::PointCloud(const PointCloud& other,
                       pc_flags::Fields copy_fields)
    : PointCloud(other.size(), ResolveFields(other, copy_fields)) {
  SetFrom(other);
}

PointCloud& PointCloud::operator=(const PointCloud& other) {
  SetFrom(other);
  return *this;
}

// Define destructor here to use complete definition of `Storage`.
PointCloud::~PointCloud() {}

void PointCloud::resize(int new_size, bool skip_initialization) {
  DRAKE_DEMAND(new_size >= 0);
  int old_size = size();
  size_ = new_size;
  storage_->resize(new_size);
  DRAKE_DEMAND(storage_->size() == new_size);
  if (new_size > old_size && !skip_initialization) {
    int size_diff = new_size - old_size;
    SetDefault(old_size, size_diff);
  }
}

void PointCloud::SetDefault(int start, int num) {
  auto set = [=](auto ref, auto value) {
    ref.middleCols(start, num).setConstant(value);
  };
  if (has_xyzs()) {
    set(mutable_xyzs(), kDefaultValue);
  }
  if (has_descriptors()) {
    set(mutable_descriptors(), kDefaultValue);
  }
}

void PointCloud::SetFrom(const PointCloud& other,
                         pc_flags::Fields fields_in,
                         bool allow_resize) {
  int old_size = size();
  int new_size = other.size();
  if (allow_resize) {
    resize(new_size);
  } else if (new_size != old_size) {
    throw std::runtime_error(
        fmt::format("SetFrom: {} != {}", new_size, old_size));
  }
  pc_flags::Fields fields_resolved =
      ResolvePointCloudPairFields(*this, other, fields_in);
  if (fields_resolved.has(pc_flags::kXYZs)) {
    mutable_xyzs() = other.xyzs();
  }
  if (fields_resolved.has_descriptor()) {
    mutable_descriptors() = other.descriptors();
  }
}

void PointCloud::Expand(
    int add_size,
    bool skip_initialization) {
  DRAKE_DEMAND(add_size >= 0);
  const int new_size = size() + add_size;
  resize(new_size, skip_initialization);
}

bool PointCloud::has_xyzs() const {
  return fields_.has(pc_flags::kXYZs);
}
Eigen::Ref<const Matrix3X<T>> PointCloud::xyzs() const {
  DRAKE_DEMAND(has_xyzs());
  return storage_->xyzs();
}
Eigen::Ref<Matrix3X<T>> PointCloud::mutable_xyzs() {
  DRAKE_DEMAND(has_xyzs());
  return storage_->xyzs();
}

bool PointCloud::has_descriptors() const {
  return fields_.has_descriptor();
}
bool PointCloud::has_descriptors(
    const pc_flags::DescriptorType& descriptor_type) const {
  return fields_.has(descriptor_type);
}
Eigen::Ref<const MatrixX<D>> PointCloud::descriptors() const {
  DRAKE_DEMAND(has_descriptors());
  return storage_->descriptors();
}
Eigen::Ref<MatrixX<D>> PointCloud::mutable_descriptors() {
  DRAKE_DEMAND(has_descriptors());
  return storage_->descriptors();
}

bool PointCloud::HasFields(
    pc_flags::Fields fields_in) const {
  DRAKE_DEMAND(!fields_in.has(pc_flags::kInherit));
  return fields_.has(fields_in);
}

void PointCloud::RequireFields(
    pc_flags::Fields fields_in) const {
  if (!HasFields(fields_in)) {
    throw std::runtime_error(
        fmt::format("PointCloud does not have expected fields.\n"
                    "Expected {}, got {}",
                    fields_in, fields()));
  }
}

bool PointCloud::HasExactFields(
    pc_flags::Fields fields_in) const {
  return fields() == fields_in;
}

void PointCloud::RequireExactFields(
    pc_flags::Fields fields_in) const {
  if (!HasExactFields(fields_in)) {
    throw std::runtime_error(
        fmt::format("PointCloud does not have the exact expected fields."
                    "\nExpected {}, got {}",
                    fields_in, fields()));
  }
}

}  // namespace perception
}  // namespace drake
