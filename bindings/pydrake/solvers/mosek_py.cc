#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_pybind.h"
#include "drake/solvers/mosek_solver.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(mosek, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = R"""(
Python bindings for the MathematicalProgram Mosek solver.

For original API documentation, please refer to the
`solvers module <https://drake.mit.edu/doxygen_cxx/group__solvers.html>`_
in the Doxygen C++ documentation.
)""";

  py::module::import("pydrake.solvers.mathematicalprogram");

  py::class_<MosekSolver, SolverInterface> cls(
      m, "MosekSolver", doc.MosekSolver.doc);
  cls.def(py::init<>(), doc.MosekSolver.ctor.doc)
      .def("set_stream_logging", &MosekSolver::set_stream_logging,
          py::arg("flag"), py::arg("log_file"),
          doc.MosekSolver.set_stream_logging.doc);
  pysolvers::BindAcquireLicense(&cls, doc.MosekSolver);
}

}  // namespace pydrake
}  // namespace drake
