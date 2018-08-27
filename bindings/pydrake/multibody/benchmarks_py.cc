#include "pybind11/eval.h"
 #include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"

namespace drake {
namespace pydrake {

using geometry::SceneGraph;
using systems::LeafSystem;

// TODO(eric.cousineau): Bind additional scalar types.
using T = double;

void init_acrobot(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody::benchmarks::acrobot;

  py::class_<AcrobotParameters>(m, "AcrobotParameters")
      .def(py::init());

  m.def("MakeAcrobotPlant",
        py::overload_cast<const AcrobotParameters&, bool, SceneGraph<double>*>(
            &MakeAcrobotPlant),
        py::arg("default_parameters"), py::arg("finalize"),
        py::arg("scene_graph") = nullptr);
}

void init_all(py::module m) {
  py::dict vars = m.attr("__dict__");
  py::exec(
      R"""(
from pydrake.multibody.benchmarks.acrobot import *
)""", py::globals(), vars);
}

PYBIND11_MODULE(benchmarks, m) {
  py::module::import("pydrake.multibody.multibody_tree");
  init_acrobot(m.def_submodule("acrobot"));

  // Pre-register this module to define an `all` module.
  py::module::import("sys").attr("modules")[
      "pydrake.multibody.benchmarks"] = m;
  init_all(m.def_submodule("all"));
}

}  // namespace pydrake
}  // namespace drake
