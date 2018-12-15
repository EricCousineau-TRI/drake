#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/deprecation_pybind.h"
#include "drake/bindings/pydrake/util/drake_optional_pybind.h"
#include "drake/bindings/pydrake/util/eigen_geometry_pybind.h"
#include "drake/bindings/pydrake/util/type_safe_index_pybind.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/weld_joint.h"

namespace drake {
namespace pydrake {
namespace {

using std::string;
using systems::Context;

// TODO(eric.cousineau): Expose available scalar types.
using T = double;

// Binds `MultibodyTreeElement` methods.
// N.B. We do this rather than inheritance because this template is more of a
// mixin than it is a parent class (since it is not used for its dynamic
// polymorphism).
// TODO(jamiesnape): Add documentation for bindings generated with this
// function.
template <typename PyClass>
void BindMultibodyTreeElementMixin(PyClass* pcls) {
  using Class = typename PyClass::type;
  auto& cls = *pcls;
  cls  // BR
      .def("get_parent_tree", &Class::get_parent_tree, py_reference_internal)
      .def("index", &Class::index)
      .def("model_instance", &Class::model_instance);
}

PYBIND11_MODULE(tree, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  // To simplify checking binding coverage, these are defined in the same order
  // as `multibody_tree_indexes.h`.
  // TODO(jamiesnape): Extract documentation automatically.
  BindTypeSafeIndex<FrameIndex>(m, "FrameIndex",
      "Type used to identify frames by index in a multibody tree system.");
  BindTypeSafeIndex<BodyIndex>(m, "BodyIndex",
      "Type used to identify bodies by index in a multibody tree system.");
  BindTypeSafeIndex<MobilizerIndex>(m, "MobilizerIndex",
      "Type used to identify mobilizers by index in a multibody tree system.");
  BindTypeSafeIndex<BodyNodeIndex>(m, "BodyNodeIndex",
      "Type used to identify tree nodes by index within a multibody tree "
      "system.");
  BindTypeSafeIndex<ForceElementIndex>(m, "ForceElementIndex",
      "Type used to identify force elements by index within a multibody tree "
      "system.");
  BindTypeSafeIndex<JointIndex>(m, "JointIndex",
      "Type used to identify joints by index within a multibody tree system.");
  BindTypeSafeIndex<JointActuatorIndex>(m, "JointActuatorIndex",
      "Type used to identify actuators by index within a multibody tree "
      "system.");
  BindTypeSafeIndex<ModelInstanceIndex>(m, "ModelInstanceIndex",
      "Type used to identify model instances by index within a multibody tree "
      "system.");
  m.def("world_index", &world_index, doc.world_index.doc);

  // Frames.
  {
    using Class = Frame<T>;
    py::class_<Class> cls(m, "Frame", doc.Frame.doc);
    BindMultibodyTreeElementMixin(&cls);
    cls  // BR
        .def("name", &Class::name, doc.Frame.name.doc)
        .def("body", &Class::body, py_reference_internal, doc.Frame.body.doc);
  }

  {
    using Class = BodyFrame<T>;
    py::class_<Class, Frame<T>> cls(m, "BodyFrame", doc.BodyFrame.doc);
    // No need to re-bind element mixins from `Frame`.
  }

  // Bodies.
  {
    using Class = Body<T>;
    py::class_<Class> cls(m, "Body", doc.Body.doc);
    BindMultibodyTreeElementMixin(&cls);
    cls  // BR
        .def("name", &Class::name, doc.Body.name.doc)
        .def("body_frame", &Class::body_frame, py_reference_internal,
            doc.Body.body_frame.doc);
  }

  {
    using Class = RigidBody<T>;
    py::class_<Class, Body<T>> cls(m, "RigidBody", doc.RigidBody.doc);
  }

  // Joints.
  {
    using Class = Joint<T>;
    py::class_<Class> cls(m, "Joint", doc.Joint.doc);
    BindMultibodyTreeElementMixin(&cls);
    cls  // BR
        .def("name", &Class::name, doc.Joint.name.doc)
        .def("parent_body", &Class::parent_body, py_reference_internal,
            doc.Joint.parent_body.doc)
        .def("child_body", &Class::child_body, py_reference_internal,
            doc.Joint.child_body.doc)
        .def("frame_on_parent", &Class::frame_on_parent, py_reference_internal,
            doc.Joint.frame_on_parent.doc)
        .def("frame_on_child", &Class::frame_on_child, py_reference_internal,
            doc.Joint.frame_on_child.doc)
        .def("position_start", &Class::position_start,
            doc.Joint.position_start.doc)
        .def("velocity_start", &Class::velocity_start,
            doc.Joint.velocity_start.doc)
        .def(
            "num_positions", &Class::num_positions, doc.Joint.num_positions.doc)
        .def("num_velocities", &Class::num_velocities,
            doc.Joint.num_velocities.doc)
        .def("lower_limits", &Class::lower_limits, doc.Joint.lower_limits.doc)
        .def("upper_limits", &Class::upper_limits, doc.Joint.upper_limits.doc);

    // Add deprecated methods.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("num_dofs", &Class::num_dofs, doc.Joint.num_dofs.doc);
#pragma GCC diagnostic pop  // pop -Wdeprecated-declarations
    cls.attr("message_num_dofs") = "Please use num_velocities().";
    DeprecateAttribute(cls, "num_dofs", cls.attr("message_num_dofs"));
  }

  {
    using Class = PrismaticJoint<T>;
    py::class_<Class, Joint<T>> cls(
        m, "PrismaticJoint", doc.PrismaticJoint.doc);
    cls  // BR
        .def("get_translation", &Class::get_translation, py::arg("context"),
            doc.PrismaticJoint.get_translation.doc)
        .def("set_translation", &Class::set_translation, py::arg("context"),
            py::arg("translation"), doc.PrismaticJoint.set_translation.doc)
        .def("get_translation_rate", &Class::get_translation_rate,
            py::arg("context"), doc.PrismaticJoint.get_translation_rate.doc)
        .def("set_translation_rate", &Class::set_translation_rate,
            py::arg("context"), py::arg("translation_dot"),
            doc.PrismaticJoint.set_translation_rate.doc);
  }

  {
    using Class = RevoluteJoint<T>;
    py::class_<Class, Joint<T>> cls(m, "RevoluteJoint", doc.RevoluteJoint.doc);
    cls  // BR
        .def(py::init<const string&, const Frame<T>&, const Frame<T>&,
                 const Vector3<T>&, double>(),
            py::arg("name"), py::arg("frame_on_parent"),
            py::arg("frame_on_child"), py::arg("axis"), py::arg("damping") = 0,
            doc.RevoluteJoint.ctor.doc_5args)
        .def("get_angle", &Class::get_angle, py::arg("context"),
            doc.RevoluteJoint.get_angle.doc)
        .def("set_angle", &Class::set_angle, py::arg("context"),
            py::arg("angle"), doc.RevoluteJoint.set_angle.doc);
  }

  {
    using Class = WeldJoint<T>;
    py::class_<Class, Joint<T>> cls(m, "WeldJoint", doc.WeldJoint.doc);
    cls  // BR
        .def(py::init<const string&, const Frame<T>&, const Frame<T>&,
                 const Isometry3<double>&>(),
            py::arg("name"), py::arg("parent_frame_P"),
            py::arg("child_frame_C"), py::arg("X_PC"),
            doc.WeldJoint.ctor.doc_4args);
  }

  // Actuators.
  {
    using Class = JointActuator<T>;
    py::class_<Class> cls(m, "JointActuator", doc.JointActuator.doc);
    BindMultibodyTreeElementMixin(&cls);
    cls  // BR
        .def("name", &Class::name, doc.JointActuator.name.doc)
        .def("joint", &Class::joint, py_reference_internal,
            doc.JointActuator.joint.doc);
  }

  // Force Elements.
  {
    using Class = ForceElement<T>;
    py::class_<Class> cls(m, "ForceElement", doc.ForceElement.doc);
    BindMultibodyTreeElementMixin(&cls);
  }

  {
    using Class = UniformGravityFieldElement<T>;
    py::class_<Class, ForceElement<T>>(
        m, "UniformGravityFieldElement", doc.UniformGravityFieldElement.doc)
        .def(py::init<Vector3<double>>(), py::arg("g_W"),
            doc.UniformGravityFieldElement.ctor.doc_1args);
  }

  // MultibodyForces
  {
    using Class = MultibodyForces<T>;
    py::class_<Class> cls(m, "MultibodyForces", doc.MultibodyForces.doc);
    cls  // BR
        .def(py::init<MultibodyTree<double>&>(), py::arg("model"),
            doc.MultibodyForces.ctor.doc_1args);
  }

  {
    using Enum = JacobianWrtVariable;
    constexpr auto& enum_doc = doc.JacobianWrtVariable;
    py::enum_<Enum> enum_py(m, "JacobianWrtVariable", enum_doc.doc);
    enum_py  // BR
        .value("kQDot", Enum::kQDot, enum_doc.kQDot.doc)
        .value("kV", Enum::kV, enum_doc.kV.doc);
  }

  // Tree.
  {
    // N.B. Pending a concrete direction on #9366, a minimal subset of the
    // `MultibodyTree` API will be exposed.
    using Class = MultibodyTree<T>;
    constexpr auto& cls_doc = doc.internal.MultibodyTree;
    py::class_<Class> cls(m, "MultibodyTree", cls_doc.doc);

    cls  // BR
        .def("CalcRelativeTransform", &Class::CalcRelativeTransform,
            py::arg("context"), py::arg("frame_A"), py::arg("frame_B"),
            cls_doc.CalcRelativeTransform.doc)
        .def("num_frames", &Class::num_frames, cls_doc.num_frames.doc)
        .def("get_body", &Class::get_body, py::arg("body_index"),
            py_reference_internal, cls_doc.get_body.doc)
        .def("get_joint", &Class::get_joint, py::arg("joint_index"),
            py_reference_internal, cls_doc.get_joint.doc)
        .def("get_joint_actuator", &Class::get_joint_actuator,
            py::arg("actuator_index"), py_reference_internal,
            cls_doc.get_joint_actuator.doc)
        .def("get_frame", &Class::get_frame, py::arg("frame_index"),
            py_reference_internal, cls_doc.get_frame.doc)
        .def("GetModelInstanceName",
            overload_cast_explicit<const string&, ModelInstanceIndex>(
                &Class::GetModelInstanceName),
            py::arg("model_instance"), py_reference_internal,
            cls_doc.GetModelInstanceName.doc)
        .def("GetPositionsAndVelocities",
            [](const MultibodyTree<T>* self,
                const Context<T>& context) -> Eigen::Ref<const VectorX<T>> {
              return self->GetPositionsAndVelocities(context);
            },
            py_reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), py::arg("context"),
            cls_doc.GetPositionsAndVelocities.doc_1args)
        .def("GetMutablePositionsAndVelocities",
            [](const MultibodyTree<T>* self,
                Context<T>* context) -> Eigen::Ref<VectorX<T>> {
              return self->GetMutablePositionsAndVelocities(context);
            },
            py_reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), py::arg("context"),
            cls_doc.GetMutablePositionsAndVelocities.doc)
        .def("CalcPointsPositions",
            [](const Class* self, const Context<T>& context,
                const Frame<T>& frame_B,
                const Eigen::Ref<const MatrixX<T>>& p_BQi,
                const Frame<T>& frame_A) {
              MatrixX<T> p_AQi(p_BQi.rows(), p_BQi.cols());
              self->CalcPointsPositions(
                  context, frame_B, p_BQi, frame_A, &p_AQi);
              return p_AQi;
            },
            py::arg("context"), py::arg("frame_B"), py::arg("p_BQi"),
            py::arg("frame_A"), cls_doc.CalcPointsPositions.doc)
        .def("CalcFrameGeometricJacobianExpressedInWorld",
            [](const Class* self, const Context<T>& context,
                const Frame<T>& frame_B, const Vector3<T>& p_BoFo_B) {
              MatrixX<T> Jv_WF(6, self->num_velocities());
              self->CalcFrameGeometricJacobianExpressedInWorld(
                  context, frame_B, p_BoFo_B, &Jv_WF);
              return Jv_WF;
            },
            py::arg("context"), py::arg("frame_B"),
            py::arg("p_BoFo_B") = Vector3<T>::Zero().eval(),
            cls_doc.CalcFrameGeometricJacobianExpressedInWorld.doc)
        .def("CalcInverseDynamics",
            overload_cast_explicit<VectorX<T>, const Context<T>&,
                const VectorX<T>&, const MultibodyForces<T>&>(
                &Class::CalcInverseDynamics),
            py::arg("context"), py::arg("known_vdot"),
            py::arg("external_forces"), cls_doc.CalcInverseDynamics.doc_3args)
        .def("SetFreeBodyPoseOrThrow",
            overload_cast_explicit<void, const Body<T>&, const Isometry3<T>&,
                Context<T>*>(&Class::SetFreeBodyPoseOrThrow),
            py::arg("body"), py::arg("X_WB"), py::arg("context"),
            cls_doc.SetFreeBodyPoseOrThrow.doc_3args)
        .def("GetPositionsFromArray", &Class::GetPositionsFromArray,
            py::arg("model_instance"), py::arg("q_array"),
            cls_doc.get_positions_from_array.doc)
        .def("GetVelocitiesFromArray", &Class::GetVelocitiesFromArray,
            py::arg("model_instance"), py::arg("v_array"),
            cls_doc.get_velocities_from_array.doc)
        .def("SetFreeBodySpatialVelocityOrThrow",
            [](const Class* self, const Body<T>& body,
                const SpatialVelocity<T>& V_WB, Context<T>* context) {
              self->SetFreeBodySpatialVelocityOrThrow(body, V_WB, context);
            },
            py::arg("body"), py::arg("V_WB"), py::arg("context"),
            cls_doc.SetFreeBodySpatialVelocityOrThrow.doc_3args)
        .def("CalcAllBodySpatialVelocitiesInWorld",
            [](const Class* self, const Context<T>& context) {
              std::vector<SpatialVelocity<T>> V_WB;
              self->CalcAllBodySpatialVelocitiesInWorld(context, &V_WB);
              return V_WB;
            },
            py::arg("context"), cls_doc.CalcAllBodySpatialVelocitiesInWorld.doc)
        .def("EvalBodyPoseInWorld",
            [](const Class* self, const Context<T>& context,
                const Body<T>& body_B) {
              return self->EvalBodyPoseInWorld(context, body_B);
            },
            py::arg("context"), py::arg("body"),
            cls_doc.EvalBodyPoseInWorld.doc)
        .def("EvalBodySpatialVelocityInWorld",
            [](const Class* self, const Context<T>& context,
                const Body<T>& body_B) {
              return self->EvalBodySpatialVelocityInWorld(context, body_B);
            },
            py::arg("context"), py::arg("body"),
            cls_doc.EvalBodySpatialVelocityInWorld.doc)
        .def("CalcAllBodyPosesInWorld",
            [](const Class* self, const Context<T>& context) {
              std::vector<Isometry3<T>> X_WB;
              self->CalcAllBodyPosesInWorld(context, &X_WB);
              return X_WB;
            },
            py::arg("context"), cls_doc.CalcAllBodyPosesInWorld.doc)
        .def("CalcMassMatrixViaInverseDynamics",
            [](const Class* self, const Context<T>& context) {
              MatrixX<T> H;
              const int n = self->num_velocities();
              H.resize(n, n);
              self->CalcMassMatrixViaInverseDynamics(context, &H);
              return H;
            },
            py::arg("context"))
        .def("CalcBiasTerm",
            [](const Class* self, const Context<T>& context) {
              VectorX<T> Cv;
              const int n = self->num_velocities();
              Cv.resize(n);
              self->CalcBiasTerm(context, &Cv);
              return Cv;
            },
            py::arg("context"), cls_doc.CalcBiasTerm.doc);
    // Add deprecated methods.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("get_multibody_state_vector",
        [](const MultibodyTree<T>* self,
            const Context<T>& context) -> Eigen::Ref<const VectorX<T>> {
          return self->get_multibody_state_vector(context);
        },
        py_reference,
        // Keep alive, ownership: `return` keeps `Context` alive.
        py::keep_alive<0, 2>(), py::arg("context"),
        cls_doc.get_multibody_state_vector.doc_1args);
    cls.def("get_mutable_multibody_state_vector",
        [](const MultibodyTree<T>* self,
            Context<T>* context) -> Eigen::Ref<VectorX<T>> {
          return self->get_mutable_multibody_state_vector(context);
        },
        py_reference,
        // Keep alive, ownership: `return` keeps `Context` alive.
        py::keep_alive<0, 2>(), py::arg("context"),
        cls_doc.get_mutable_multibody_state_vector.doc);
#pragma GCC diagnostic pop  // pop -Wdeprecated-declarations
    cls.attr("message_get_mutable_multibody_state_vector") =
        "Please use GetMutablePositionsAndVelocities().";
    DeprecateAttribute(cls, "get_mutable_multibody_state_vector",
        cls.attr("message_get_mutable_multibody_state_vector"));
    cls.attr("message_get_multibody_state_vector") =
        "Please use GetPositionsAndVelocities().";
    DeprecateAttribute(cls, "get_multibody_state_vector",
        cls.attr("message_get_multibody_state_vector"));
  }
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
