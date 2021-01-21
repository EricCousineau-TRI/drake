#pragma once

#include "drake/common/drake_deprecated.h"
#include "drake/common/unused.h"

namespace drake {
namespace example_class {

/// Example class.
class ExampleCppClass {
 public:
  /// Good constructor.
  ExampleCppClass() {}

  /// ctor(int) will be deprecated.
  explicit ExampleCppClass(int x) { unused(x); }

  /// ctor(double) will be deprecated.
  explicit ExampleCppClass(double y) { unused(y); }

  /// This will be full deprecated.
  void DeprecatedMethod() {}

  /// Good overload.
  void overload() {}

  /// This will be deprecated.
  void overload(int x) { unused(x); }

  /// Not yet deprecated.
  int deprecated_aliased_prop{};
};

/// Simple struct.
struct ExampleCppStruct {
  int i{};
  int j{};
};

}  // namespace example_class
}  // namespace drake
