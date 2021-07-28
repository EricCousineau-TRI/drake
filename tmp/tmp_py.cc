#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace pydrake {

struct NonCopyable {
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NonCopyable)
  NonCopyable() {}
};

class Container {
 public:
  const NonCopyable& get_noncopyable() const { return noncopyable_; }
 private:
  NonCopyable noncopyable_;
};

void BindStuff(py::module m) {
  auto wrapped = WrapDeprecated("Deprecated", &Container::get_noncopyable);
  py::class_<Container>(m, "Container").def("get_noncopyable", wrapped);
}

PYBIND11_MODULE(tmp, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  BindStuff(m);
}

}  // namespace pydrake
}  // namespace drake
