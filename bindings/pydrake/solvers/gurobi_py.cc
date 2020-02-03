#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_pybind.h"
#include "drake/solvers/gurobi_solver.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(gurobi, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = R"""(
Python bindings for the MathematicalProgram Gurobi solver.

For original API documentation, please refer to the
`solvers module <https://drake.mit.edu/doxygen_cxx/group__solvers.html>`_
in the Doxygen C++ documentation.
)""";

  py::module::import("pydrake.solvers.mathematicalprogram");

  py::class_<GurobiSolver, SolverInterface> cls(
      m, "GurobiSolver", doc.GurobiSolver.doc);
  cls.def(py::init<>(), doc.GurobiSolver.ctor.doc);
  pysolvers::BindAcquireLicense(&cls, doc.GurobiSolver);
}

}  // namespace pydrake
}  // namespace drake
