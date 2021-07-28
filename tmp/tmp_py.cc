#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace pydrake {

class FooSystem : public systems::LeafSystem<double> {
 public:
  FooSystem() : systems::LeafSystem<double>() {
    port1_ =
        this->DeclareAbstractOutputPort("port1", &FooSystem::Calc1).get_index();
    port2_ =
        this->DeclareAbstractOutputPort("port2", &FooSystem::Calc1).get_index();
  }
  const systems::OutputPort<double>& get_output_port1() const {
    return systems::System<double>::get_output_port(port1_);
  }
  const systems::OutputPort<double>& get_output_port2() const {
    return systems::System<double>::get_output_port(port2_);
  }

 private:
  void Calc1(const systems::Context<double>& context, double* output) const {
    *output = context.get_time() + 1.0;
  }
  void Calc2(const systems::Context<double>& context, double* output) const {
    *output = context.get_time() + 2.0;
  }
  int port1_{-1};
  int port2_{-1};
};

void BindStuff(py::module m) {
  using Class = FooSystem;
  py::class_<Class> cls(m, "FooSystem", "class doc");
  cls  // BR
      .def(py::init(), "Constructor")
      .def("get_output_port1", &Class::get_output_port1,
          py_rvp::reference_internal, "Port 1 doc")
      .def("get_output_port2", 
          WrapDeprecated("Deprecated 2038-01-01", &Class::get_output_port2),
          py_rvp::reference_internal, "Port 2 doc");
}

PYBIND11_MODULE(tmp, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  BindStuff(m);
}

}  // namespace pydrake
}  // namespace drake
