#include "drake/bindings/pydrake/util/cpp_types_pybind.h"

// @file
// Tests the public interfaces in `cpp_types.py` and `cpp_types_pybind.h`.

#include <string>

#include <gtest/gtest.h>
#include <pybind11/embed.h>
#include <pybind11/eval.h>
#include <pybind11/pybind11.h>

using std::string;

namespace drake {
namespace pydrake {

template <typename T>
bool CheckPyType(const string& py_expr_expected) {
  return GetPyType<T>().is(py::eval(py_expr_expected.c_str()));
}

bool PyEquals(py::object lhs, py::object rhs) {
  return lhs.attr("__eq__")(rhs).cast<bool>();
}

template <int Value>
using int_constant = std::integral_constant<int, Value>;

template <int Value>
using uint_constant = std::integral_constant<uint, Value>;

struct CustomCppType {};

GTEST_TEST(CppTypesTest, InCpp) {
  // Define custom class only once here.
  py::module m("__main__");
  py::globals()["np"] = py::module::import("numpy");

  py::class_<CustomCppType>(m, "CustomCppType");

  // Check C++ behavior.
  ASSERT_TRUE(CheckPyType<bool>("bool"));
  ASSERT_TRUE(CheckPyType<std::string>("str"));
  ASSERT_TRUE(CheckPyType<double>("float"));
  ASSERT_TRUE(CheckPyType<float>("np.float32"));
  ASSERT_TRUE(CheckPyType<int>("int"));
  ASSERT_TRUE(CheckPyType<py::object>("object"));

  // Custom types.
  ASSERT_TRUE(CheckPyType<CustomCppType>("CustomCppType"));
  ASSERT_TRUE(PyEquals(GetPyTypes<int, bool>(), py::eval("int, bool")));

  // Literals parameters.
  ASSERT_TRUE(CheckPyType<std::true_type>("True"));
  ASSERT_TRUE(CheckPyType<int_constant<-1>>("-1"));
  ASSERT_TRUE(CheckPyType<uint_constant<1>>("1"));
}

int main(int argc, char** argv) {
  // Reconstructing `scoped_interpreter` multiple times (e.g. via `SetUp()`)
  // while *also* importing `numpy` wreaks havoc.
  py::scoped_interpreter guard;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace pydrake
}  // namespace drake

int main(int argc, char** argv) {
  return drake::pydrake::main(argc, argv);
}
