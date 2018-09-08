#include <gtest/gtest.h>

// Ensure we can include files from their old path.
#include "drake/bindings/pydrake/util/cpp_param_pybind.h"
#include "drake/bindings/pydrake/util/cpp_template_pybind.h"
#include "drake/bindings/pydrake/util/deprecation_pybind.h"
#include "drake/bindings/pydrake/util/drake_optional_pybind.h"
#include "drake/bindings/pydrake/util/eigen_geometry_pybind.h"
#include "drake/bindings/pydrake/util/type_pack.h"
#include "drake/bindings/pydrake/util/type_safe_index.h"
#include "drake/bindings/pydrake/util/wrap_function.h"
#include "drake/bindings/pydrake/util/wrap_pybind.h"


namespace drake {
namespace pydrake {
namespace {

GTEST_TEST(AliasCcTest, Dummy) {}

}  // namespace
}  // namespace pydrake
}  // namespace drake
