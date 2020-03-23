#include <Eigen/Dense>

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

namespace {

namespace py = pybind11;

using Eigen::VectorXd;

VectorXd pass_through(VectorXd x) {
  return x;
}

PYBIND11_MODULE(bleh_lib, m) {
  m.def("pass_through", &pass_through, py::arg("x"));
}

}  // namespace
