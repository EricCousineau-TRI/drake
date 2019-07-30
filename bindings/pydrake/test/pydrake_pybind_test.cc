/// @file
/// Test binding helper methods in `pydrake_pybind_test`.
#include "drake/bindings/pydrake/pydrake_pybind.h"

#include <string>

#include <gtest/gtest.h>
#include "pybind11/embed.h"
#include "pybind11/eval.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/test/test_util_pybind.h"

namespace drake {
namespace pydrake {
namespace {

void PyExpectTrue(py::module m, const char* expr) {
  const bool value =
      py::eval(expr, py::globals(), m.attr("__dict__")).cast<bool>();
  EXPECT_TRUE(value) << expr;
}

struct TestCopyAndDeepCopy {
  TestCopyAndDeepCopy(const TestCopyAndDeepCopy&) = default;
  int value{};
  bool operator==(const TestCopyAndDeepCopy& other) const {
    return value == other.value;
  }
};

GTEST_TEST(PydrakePybindTest, DefCopyAndDeepCopy) {
  py::module m("test");
  {
    using Class = TestCopyAndDeepCopy;
    py::class_<Class> cls(m, "TestCopyAndDeepCopy");
    cls  // BR
        .def(py::init([](int value) { return Class{value}; }))
        .def(py::self == py::self);
    DefCopyAndDeepCopy(&cls);
  }

  PyExpectTrue(m, "check_copy(copy.copy, TestCopyAndDeepCopy(10))");
  PyExpectTrue(m, "check_copy(copy.deepcopy, TestCopyAndDeepCopy(20))");
}

class TestClone {
 public:
  explicit TestClone(int value) : value_(value) {}
  TestClone(TestClone&&) = delete;

  std::unique_ptr<TestClone> Clone() const {
    return std::unique_ptr<TestClone>(new TestClone(*this));
  }

  bool operator==(const TestClone& other) const {
    return value_ == other.value_;
  }

 private:
  TestClone(const TestClone&) = default;
  int value_{};
};

GTEST_TEST(PydrakePybindTest, DefClone) {
  py::module m("test");
  {
    using Class = TestClone;
    py::class_<Class> cls(m, "TestClone");
    cls  // BR
        .def(py::init<double>())
        .def(py::self == py::self);
    DefClone(&cls);
  }

  PyExpectTrue(m, "check_copy(TestClone.Clone, TestClone(5))");
  PyExpectTrue(m, "check_copy(copy.copy, TestClone(10))");
  PyExpectTrue(m, "check_copy(copy.deepcopy, TestClone(20))");
}

struct TestParamInit {
  int a{0};
  int b{1};
};

GTEST_TEST(PydrakePybindTest, ParamInit) {
  py::module m("test");
  {
    using Class = TestParamInit;
    py::class_<Class>(m, "TestParamInit")
        .def(ParamInit<Class>())
        .def_readwrite("a", &Class::a)
        .def_readwrite("b", &Class::b)
        .def("as_tuple",
            [](const Class& self) { return py::make_tuple(self.a, self.b); });
  }

  PyExpectTrue(m, "TestParamInit().as_tuple() == (0, 1)");
  PyExpectTrue(m, "TestParamInit(a=10).as_tuple() == (10, 1)");
  PyExpectTrue(m, "TestParamInit(b=20).as_tuple() == (0, 20)");
  PyExpectTrue(m, "TestParamInit(a=10, b=20).as_tuple() == (10, 20)");
}

int DoMain(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  // Reconstructing `scoped_interpreter` multiple times (e.g. via `SetUp()`)
  // while *also* importing `numpy` wreaks havoc.
  py::scoped_interpreter guard;
  // Define nominal scope, and use a useful name for `ExecuteExtraPythonCode`
  // below.
  py::module m("pydrake.test.pydrake_pybind_test");
  // Test coverage and use this method.
  ExecuteExtraPythonCode(m);
  test::SynchronizeGlobalsForPython3(m);
  return RUN_ALL_TESTS();
}

}  // namespace
}  // namespace pydrake
}  // namespace drake

int main(int argc, char** argv) {
  return drake::pydrake::DoMain(argc, argv);
}
