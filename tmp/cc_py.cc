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

// Try to replicate signature as in #20516.

PYBIND11_MODULE(cc, m) {
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
