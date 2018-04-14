#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/wrap_pybind.h"

using Eigen::AutoDiffScalar;
using std::sin;
using std::cos;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(_autodiffutils_py, m) {
  m.doc() = "Bindings for Eigen AutoDiff Scalars";

  // Install NumPy warning filtres.
  // N.B. This may interfere with other code, but until that is a confirmed
  // issue, we should agressively try to avoid these warnings.
  py::module::import("pydrake.util.deprecation")
      .attr("install_numpy_warning_filters")();

  py::dtype_user<AutoDiffXd> autodiff(m, "AutoDiffXd");
  autodiff
    .def(py::init<double>())
    .def(py::init<const double&, const Eigen::VectorXd&>())
    // Casting
    // - Upcasting can be implicit, especially for matrix multiplication.
    .def_loop_cast([](double x) -> AutoDiffXd { return x; }, true)
    .def_loop_cast([](int in) -> AutoDiffXd { return in; }, true)
    // See https://github.com/numpy/numpy/issues/10904 for next 2 casts.
    // NOLINTNEXTLINE(runtime/int): Use platform-dependent name.
    .def_loop_cast([](long in) -> AutoDiffXd { return in; }, true)
    .def_loop_cast([](bool in) -> AutoDiffXd { return in; }, true)
    // - Downcasting must be explicit, to prevent inadvertent information loss.
    .def_loop_cast([](const AutoDiffXd& self) -> double {
      return self.value();
    })
    // General methods.
    .def("value", [](const AutoDiffXd& self) {
      return self.value();
    })
    .def("derivatives", [](const AutoDiffXd& self) {
      return self.derivatives();
    })
    .def("__str__", [](const AutoDiffXd& self) {
      return py::str("AD{{{}, nderiv={}}}").format(
          self.value(), self.derivatives().size());
    })
    .def("__repr__", [](const AutoDiffXd& self) {
      return py::str("<AutoDiffXd {} nderiv={}>").format(
          self.value(), self.derivatives().size());
    })
    // Arithmetic
    .def_loop(-py::self)
    .def_loop(py::self + py::self)
    .def_loop(py::self + double())
    .def_loop(double() + py::self)
    .def_loop(py::self - py::self)
    .def_loop(py::self - double())
    .def_loop(double() - py::self)
    .def_loop(py::self * py::self)
    .def_loop(py::self * double())
    .def_loop(double() * py::self)
    .def_loop(py::self / py::self)
    .def_loop(py::self / double())
    .def_loop(double() / py::self)
    // Logical comparison
    .def_loop(py::self == py::self)
    .def_loop(py::self == double())
    .def_loop(py::self != py::self)
    .def_loop(py::self != double())
    // .def_loop(double() != py::self)
    .def_loop(py::self < py::self)
    .def_loop(py::self < double())
    .def_loop(double() < py::self)
    .def_loop(py::self <= py::self)
    .def_loop(py::self <= double())
    .def_loop(double() <= py::self)
    .def_loop(py::self > py::self)
    .def_loop(py::self > double())
    .def_loop(double() > py::self)
    .def_loop(py::self >= py::self)
    .def_loop(py::self >= double())
    .def_loop(double() >= py::self)
    // Dot-product
    .def_loop(py::dtype_method::dot())
    // N.B. Certain versions of NumPy require `square` be specifically
    // defined.
    .def_loop("square", [](const AutoDiffXd& self) {
      return self * self;
    });

  // Add overloads for `math` functions.
  auto math = py::module::import("pydrake.math");
  UfuncMirrorDef<decltype(autodiff)>(&autodiff, math)
    .def_loop("__pow__", "pow",
         [](const AutoDiffXd& base, int exponent) {
           return pow(base, exponent);
         })
    .def_loop("__abs__", "abs", [](const AutoDiffXd& x) { return abs(x); })
    .def_loop("log", [](const AutoDiffXd& x) { return log(x); })
    .def_loop("exp", [](const AutoDiffXd& x) { return exp(x); })
    .def_loop("sqrt", [](const AutoDiffXd& x) { return sqrt(x); })
    .def_loop("sin", [](const AutoDiffXd& x) { return sin(x); })
    .def_loop("cos", [](const AutoDiffXd& x) { return cos(x); })
    .def_loop("tan", [](const AutoDiffXd& x) { return tan(x); })
    .def_loop("arcsin", "asin", [](const AutoDiffXd& x) { return asin(x); })
    .def_loop("arccos", "acos", [](const AutoDiffXd& x) { return acos(x); })
    .def_loop("arctan2", "atan2",
        [](const AutoDiffXd& y, const AutoDiffXd& x) {
            return atan2(y, x);
        })
    .def_loop("sinh", [](const AutoDiffXd& x) { return sinh(x); })
    .def_loop("cosh", [](const AutoDiffXd& x) { return cosh(x); })
    .def_loop("tanh", [](const AutoDiffXd& x) { return tanh(x); })
    .def_loop("fmin", "min",
        [](const AutoDiffXd& x, const AutoDiffXd& y) {
            return min(x, y);
        })
    .def_loop("fmax", "max",
        [](const AutoDiffXd& x, const AutoDiffXd& y) {
            return max(x, y);
        })
    .def_loop("ceil", [](const AutoDiffXd& x) { return ceil(x); })
    .def_loop("floor", [](const AutoDiffXd& x) { return floor(x); });
}

}  // namespace pydrake
}  // namespace drake
