#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

namespace py = pybind11;

namespace {

PYBIND11_MODULE(convert_py, m) {
  m.def("pass_through", [](const Eigen::Ref<const Eigen::MatrixXd>& A) {
    return A;
  });
}

}
