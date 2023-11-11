/// Confirms that drake#20516 is no longer an issue.
// TODO(eric.cousineau): Move this test to pybind11 fork once I've figured out
// why the test does not fail under its testing setup:
// https://github.com/RobotLocomotion/drake/issues/20516#issuecomment-1806546549

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

namespace py = pybind11;

namespace drake {

struct MyScalar {
  int value{};
};

}  // namespace drake

PYBIND11_NUMPY_OBJECT_DTYPE(drake::MyScalar);

namespace drake {
namespace {

std::string AcceptMatrixDense(const Eigen::MatrixXd&) {
  return "dense";
}

std::string AcceptMatrixSparse(const Eigen::SparseMatrix<double>&) {
  return "sparse";
}

using VectorXMyScalar = Eigen::Matrix<MyScalar, Eigen::Dynamic, 1>;

std::string AcceptMatrixAndObjectDense(
    const Eigen::MatrixXd&, const VectorXMyScalar&) {
  return "dense";
}

std::string AcceptMatrixAndObjectSparse(
    const Eigen::SparseMatrix<double>&, const VectorXMyScalar&) {
  return "sparse";
}

PYBIND11_MODULE(eigen_dtype_object_noconvert, m) {
  m.def("AcceptMatrix", &AcceptMatrixDense);
  m.def("AcceptMatrix", &AcceptMatrixSparse);

  py::class_<MyScalar>(m, "MyScalar")
      .def(py::init<int>(), py::arg("value"))
      .def_readwrite("value", &MyScalar::value);

  m.def("AcceptMatrixAndObject", &AcceptMatrixAndObjectSparse);
  m.def("AcceptMatrixAndObject", &AcceptMatrixAndObjectDense);
}

}  // namespace
}  // namespace drake
