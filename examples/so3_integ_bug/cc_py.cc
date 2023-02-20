#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/tree/quaternion_rate.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(cc, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;

  py::module::import("pydrake.common.eigen_geometry");

  {
    using Class = QuaternionRate<double>;
    py::class_<Class>(m, "QuaternionRate")
        .def_static(
            "AngularVelocityToQuaternionRateMatrix",
            &Class::AngularVelocityToQuaternionRateMatrix)
        .def_static(
            "QuaternionRateToAngularVelocityMatrix",
            &Class::QuaternionRateToAngularVelocityMatrix);
  }
}

}  // namespace pydrake
}  // namespace drake
