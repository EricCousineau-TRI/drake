#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/sorted_pair_pybind.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/optimization/static_equilibrium_constraint.h"
#include "drake/multibody/optimization/static_equilibrium_problem.h"

using drake::multibody::internal::GeometryPairContactWrenchEvaluatorBinding;

namespace pybind11 {
namespace detail {

// Artisinal caster to avoid exposing
// `GeometryPairContactWrenchEvaluatorBinding` as public API.
template <typename T>
struct type_caster<GeometryPairContactWrenchEvaluatorBinding> {
  using Type = GeometryPairContactWrenchEvaluatorBinding;
  // N.B. This macro assumes placement in `pybind11::detail`.
  PYBIND11_TYPE_CASTER(Type, _("GeometryPairContactWrenchEvaluatorBinding"));

  bool load(handle src, bool convert) {
    throw std::runtime_error("Unsupported");
  }

  static handle cast(Type src, return_value_policy, handle) {
    return make_tuple(
      cast(src.lambda_indices_in_all_lambda),
      cast(src.contact_wrench_evaluator),
    );
  }
};

}  // namespace detail
}  // namespace pybind11

namespace drake {
namespace pydrake {

namespace {
PYBIND11_MODULE(optimization, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  m.doc() = "Optimization module for MultibodyPlant motion planning";

  py::module::import("pydrake.math");
  py::module::import("pydrake.multibody.plant");
  py::module::import("pydrake.solvers.mathematicalprogram");

  {
    py::class_<ContactWrench>(m, "ContactWrench", doc.ContactWrench.doc)
        .def_readonly("bodyA_index", &ContactWrench::bodyA_index,
            doc.ContactWrench.bodyA_index.doc)
        .def_readonly("bodyB_index", &ContactWrench::bodyB_index,
            doc.ContactWrench.bodyB_index.doc)
        .def_readonly(
            "p_WCb_W", &ContactWrench::p_WCb_W, doc.ContactWrench.p_WCb_W.doc)
        .def_readonly(
            "F_Cb_W", &ContactWrench::F_Cb_W, doc.ContactWrench.F_Cb_W.doc);
    AddValueInstantiation<ContactWrench>(m);
  }


  {
    using Class = StaticEquilibriumConstraint;
    constexpr auto& cls_doc = doc.StaticEquilibriumConstraint;
    
    py::class_<Class, solvers::Constraint>(m, "StaticEquilibriumConstraint", cls_doc.doc)
        .def_static("MakeBinding",
            static_cast<solvers::Binding<Class> (*)(const MultibodyPlant<AutoDiffXd>*,
                systems::Context<AutoDiffXd>*,
                const std::vector<
                    std::pair<std::shared_ptr<ContactWrenchEvaluator>,
                        VectorX<symbolic::Variable>>>&,
                const Eigen::Ref<const VectorX<symbolic::Variable>>&,
                const Eigen::Ref<const VectorX<symbolic::Variable>>&)>(
                &Class::MakeBinding),
            py::arg("plant"), py::arg("context"),
            py::arg("contact_wrench_evaluators_and_lambda"), py::arg("q_vars"),
            py::arg("u_vars"),
            // Keep alive, reference: `plant` keeps `return` alive.
            py::keep_alive<1, 0>(),
            // Keep alive, reference: `context` keeps `return` alive.
            py::keep_alive<2, 0>(), cls_doc.MakeBinding.doc)
        .def("contact_pair_to_wrench_evaluator",
           static_cast<
              const std::map<SortedPair<geometry::GeometryId>,
                 GeometryPairContactWrenchEvaluatorBinding>&&
                 (Class::*)() const>(
               &Class::contact_pair_to_wrench_evaluator),
            cls_doc.contact_pair_to_wrench_evaluator.doc);
  }

  {
    using Class = StaticEquilibriumProblem;
    constexpr auto& cls_doc = doc.StaticEquilibriumProblem;
    py::class_<Class>(m, "StaticEquilibriumProblem", cls_doc.doc)
        .def(py::init<const MultibodyPlant<AutoDiffXd>*,
                 systems::Context<AutoDiffXd>*,
                 const std::set<
                     std::pair<geometry::GeometryId, geometry::GeometryId>>&>(),
            py::arg("plant"), py::arg("context"),
            py::arg("ignored_collision_pairs"),
            // Keep alive, reference: `self` keeps `plant` and `context` alive.
            py::keep_alive<1, 2>(), py::keep_alive<1, 3>(), cls_doc.ctor.doc)
        .def("get_mutable_prog", &Class::get_mutable_prog,
            py_rvp::reference_internal, cls_doc.get_mutable_prog.doc)
        .def("prog", &Class::prog, py_rvp::reference_internal, cls_doc.prog.doc)
        .def("q_vars", &Class::q_vars, cls_doc.q_vars.doc)
        .def("u_vars", &Class::u_vars, cls_doc.u_vars.doc)
        .def("GetContactWrenchSolution", &Class::GetContactWrenchSolution,
            py::arg("result"), cls_doc.GetContactWrenchSolution.doc)
        .def("UpdateComplementarityTolerance",
            &Class::UpdateComplementarityTolerance, py::arg("tol"),
            cls_doc.UpdateComplementarityTolerance.doc);
  }
}
}  // namespace
}  // namespace pydrake
}  // namespace drake
