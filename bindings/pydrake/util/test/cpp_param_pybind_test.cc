#include "drake/bindings/pydrake/util/cpp_param_pybind.h"

// @file
// Tests the public interfaces in `cpp_param.py` and `cpp_param_pybind.h`.

#include <string>

#include <gtest/gtest.h>
#include <pybind11/embed.h>
#include <pybind11/eval.h>
#include <pybind11/pybind11.h>

using std::string;

namespace drake {
namespace pydrake {

// Ensures that the type `T` maps to the expression in `py_expr_expected`.
template <typename T>
bool CheckPyParam(const string& py_expr_expected, type_pack<T> = {}) {
  py::object actual = GetPyParam<T>()[0];
  py::object expected = py::eval(py_expr_expected.c_str());
  return actual.is(expected);
}

GTEST_TEST(CppParamTest, PrimitiveTypes) {
  // Tests primitive types that are not expose directly via `pybind11`, thus
  // needing custom registration.
  // This follows the ordering in `cpp_param_pybind.cc`,
  // `RegisterCommon`.
  ASSERT_TRUE(CheckPyParam<bool>("bool"));
  ASSERT_TRUE(CheckPyParam<std::string>("str"));
  ASSERT_TRUE(CheckPyParam<double>("float"));
  ASSERT_TRUE(CheckPyParam<float>("np.float32"));
  ASSERT_TRUE(CheckPyParam<int>("int"));
  ASSERT_TRUE(CheckPyParam<uint32_t>("np.uint32"));
  ASSERT_TRUE(CheckPyParam<int64_t>("np.int64"));
  ASSERT_TRUE(CheckPyParam<py::object>("object"));
}

// Dummy type.
struct CustomCppType {};

GTEST_TEST(CppParamTest, CustomTypes) {
  // Tests types that are C++ types registered with `pybind11`.
  ASSERT_TRUE(CheckPyParam<CustomCppType>("CustomCppType"));
}

template <typename T, T Value>
using constant = std::integral_constant<T, Value>;

GTEST_TEST(CppParamTest, LiteralTypes) {
  // Tests that literal types are mapped to literals in Python.
  ASSERT_TRUE(CheckPyParam<std::true_type>("True"));
  ASSERT_TRUE((CheckPyParam<constant<int, -1>>("-1")));
  ASSERT_TRUE((CheckPyParam<constant<uint, 1>>("1")));
}

// Compare two Python objects directly.
bool PyEquals(py::object lhs, py::object rhs) {
  return lhs.attr("__eq__")(rhs).cast<bool>();
}

GTEST_TEST(CppParamTest, Packs) {
  // Tests that type packs are properly interpreted.
  ASSERT_TRUE(PyEquals(GetPyParam<int, bool>(), py::eval("int, bool")));
}

int main(int argc, char** argv) {
  // Reconstructing `scoped_interpreter` multiple times (e.g. via `SetUp()`)
  // while *also* importing `numpy` wreaks havoc.
  py::scoped_interpreter guard;

  // Define common scope, import numpy for use in `eval`.
  py::module m("__main__");
  py::globals()["np"] = py::module::import("numpy");

  // Define custom class only once here.
  py::class_<CustomCppType>(m, "CustomCppType");

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace pydrake
}  // namespace drake

int main(int argc, char** argv) {
  return drake::pydrake::main(argc, argv);
}
