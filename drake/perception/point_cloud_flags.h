#pragma once

#include <iostream>
#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace perception {

/// Point cloud flags.
namespace pc_flags {

typedef int BaseFieldT;
/// Indicates the data the point cloud stores.
enum BaseField : int {
  kNone = 0,
  /// Inherit other fields. May imply an intersection of all
  /// compatible descriptors.
  kInherit = 1 << 0,
  /// XYZ point in Cartesian space.
  kXYZs = 1 << 1,
};

/// Describes an descriptor field with a name and the descriptor's size.
///
/// @note This is defined as follows to enable an open set of descriptors, but
/// ensure that these descriptor types are appropriately matched.
/// As `PointCloud` evolves and more algorithms are mapped into Drake,
/// promoting an descriptor field to a proper field should be considered if (a)
/// it is used frequently enough AND (b) if it is often used in conjunction
/// with other fields.
class DescriptorType final {
 public:
  DescriptorType(int size, const std::string& name)
      : size_(size),
        name_(name) {}

  DescriptorType(const DescriptorType&) = default;

  // Set up defaults.
  DescriptorType(DescriptorType&&) = delete;
  DescriptorType& operator=(const DescriptorType&) = default;
  DescriptorType& operator=(DescriptorType&&) = default;

  inline int size() const { return size_; }
  inline const std::string& name() const { return name_; }
  inline bool operator==(const DescriptorType& other) const {
    return size_ == other.size_ && name_ == other.name_;
  }
  inline bool operator!=(const DescriptorType& other) const {
    return !(*this == other);
  }

 private:
  int size_{};
  std::string name_;
};

/// No descriptor.
const DescriptorType kDescriptorNone(0, "kDescriptorNone");
/// Curvature.
const DescriptorType kDescriptorCurvature(1, "kDescriptorCurvature");
/// Point-feature-histogram.
const DescriptorType kDescriptorFPFH(33, "kDescriptorFPFH");

/**
 * Allows combination of `BaseField` and `DescriptorType` for a `PointCloud`.
 *
 * This provides the mechanism to use basic union bit-mask operators (|) to
 * convey a combination of fields.
 */
class Fields {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Fields)

  // NOLINTNEXTLINE(runtime/explicit): This conversion is desirable.
  Fields(BaseFieldT base_fields)
      : base_fields_(base_fields) {
    if (base_fields < 0 || base_fields >= (kXYZs << 1))
      throw std::runtime_error("Invalid BaseField specified.");
  }

  // NOLINTNEXTLINE(runtime/explicit): This conversion is desirable.
  Fields(const DescriptorType& descriptor_type)
      : descriptor_type_(descriptor_type) {}

  BaseFieldT base_fields() const { return base_fields_; }

  bool has_base_fields() const {
    return base_fields_ != kNone;
  }

  const DescriptorType& descriptor_type() const { return descriptor_type_; }

  bool has_descriptor() const {
    return descriptor_type_ != kDescriptorNone;
  }

  /// Provides union.
  Fields& operator|=(const Fields& rhs) {
    base_fields_ = base_fields_ | rhs.base_fields_;
    if (has_descriptor())
      throw std::runtime_error(
          "Cannot have multiple Descriptor flags. "
          "Can only add flags iff (!rhs.has_descriptor()).");
    descriptor_type_ = rhs.descriptor_type_;
    return *this;
  }

  Fields operator|(const Fields& rhs) const {
    return Fields(*this) |= rhs;
  }

  Fields& operator&=(const Fields& rhs) {
    base_fields_ &= rhs.base_fields_;
    if (descriptor_type_ != rhs.descriptor_type_) {
      descriptor_type_ = kDescriptorNone;
    }
    return *this;
  }

  /// Provides intersection.
  Fields operator&(const Fields& rhs) const {
    return Fields(*this) &= rhs;
  }

  bool has_any() const {
    return has_base_fields() || has_descriptor();
  }

  // NOTE: Cannot use operator bool() as this creates conflicts when
  // attempting to use other flags.
  bool has(const Fields& rhs) const {
    return (*this & rhs) == rhs;
  }

  bool operator==(const Fields& rhs) const {
    return (base_fields_ == rhs.base_fields_
            && descriptor_type_ == rhs.descriptor_type_);
  }

  friend std::ostream& operator<<(std::ostream& os, const Fields& rhs);

  bool operator!=(const Fields& rhs) const {
    return !(*this == rhs);
  }

 private:
  // TODO(eric.cousineau): Use `optional` to avoid the need for `none` objects?
  BaseFieldT base_fields_{kNone};
  DescriptorType descriptor_type_{kDescriptorNone};
};

// Make | compatible for BaseField + Descriptor. Do not use implicit conversion
// because it becomes ambiguous.
inline Fields operator|(const BaseFieldT& lhs, const DescriptorType& rhs) {
  return Fields(lhs) | Fields(rhs);
}

// Make | compatible for Descriptor + BaseField.
inline Fields operator|(const DescriptorType& lhs, const BaseFieldT& rhs) {
  return Fields(lhs) | Fields(rhs);
}

}  // namespace pc_flags

}  // namespace perception
}  // namespace drake
