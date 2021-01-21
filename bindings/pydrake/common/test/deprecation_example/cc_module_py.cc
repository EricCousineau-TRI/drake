/*
This file provides a set of examples for deprecating methods, overloads,
classes, etc.

Please review this file and the corresponding test,
`deprecation_example_test.py`.
*/

#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/test/deprecation_example/example_class.h"  // NOLINT
#include "drake/bindings/pydrake/common/test/deprecation_example/example_class_documentation.h"  // NOLINT
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace {

PYBIND11_MODULE(cc_module, m) {
  constexpr auto& doc = pydrake_doc.drake.example_class;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::example_class;

  {
    using Class = ExampleCppStruct;
    constexpr auto& cls_doc = doc.ExampleCppStruct;
    py::class_<Class> cls(m, "ExampleCppStruct", cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>(), cls_doc.doc)
        .def_readwrite("i", &Class::i)
        .def_readwrite("j", &Class::j);
  }

  {
    using Class = ExampleCppClass;
    constexpr auto& cls_doc = doc.ExampleCppClass;
    py::class_<Class> cls(m, "ExampleCppClass", cls_doc.doc);
    cls  // BR
        .def(py::init(), cls_doc.ctor.doc_0args)
        .def(py::init<int>(), py::arg("x"), cls_doc.ctor.doc_1args_x)
        .def(py::init([](double arg) { return Class(arg); }), py::arg("y"),
            cls_doc.ctor.doc_1args_y)
        .def("DeprecatedMethod", &Class::DeprecatedMethod,
            cls_doc.DeprecatedMethod.doc)
        .def("overload", py::overload_cast<>(&Class::overload),
            cls_doc.overload.doc_0args)
        .def("overload", py::overload_cast<int>(&Class::overload), py::arg("x"),
            cls_doc.overload.doc_1args)
        .def_readwrite("deprecated_aliased_prop",
            &Class::deprecated_aliased_prop,
            cls_doc.deprecated_aliased_prop.doc);
  }
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
