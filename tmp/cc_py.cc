#include <pybind11/pybind11.h>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"

namespace py = pybind11;

namespace drake {
namespace pydrake {

class MyDiagram : public systems::Diagram<double> {
 public:
  MyDiagram() {
    systems::DiagramBuilder<double> builder;
    builder.AddSystem<systems::Adder<double>>(0, 1);
    builder.BuildInto(this);
  }
};

PYBIND11_MODULE(cc, m) {
  py::module::import("pydrake.systems.framework");

  m.def("make_my_diagram_as_system", []()
      -> std::unique_ptr<systems::System<double>> {
    return std::make_unique<MyDiagram>();
  });
  m.def("make_my_diagram_as_diagram", []()
      -> std::unique_ptr<systems::Diagram<double>> {
    return std::make_unique<MyDiagram>();
  });

  m.def("as_diagram", [](systems::System<double>* system) {
    return dynamic_cast<systems::Diagram<double>*>(system);
  }, py::return_value_policy::reference, py::keep_alive<0, 1>());
}

}  // namespace
}  // namespace pydrake
