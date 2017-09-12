#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/vector_system.h"

namespace drake {
namespace systems {

/// A pass through system with input `u` and output `y = u`. This is
/// mathematically equivalent to a Gain system with its gain equal to one.
/// However this system incurs no computational cost.
/// The input to this system directly feeds through to its output.
/// This system is used, for instance, in PidController which is a Diagram
/// composed of simple framework primitives. In this case a PassThrough is used
/// to connect the exported input of the Diagram to the inputs of the Gain
/// systems for the proportioanal and integral constants of the controller. This
/// is necessary to provide an output port to which the internal Gain subsystems
/// connect. In this case the PassThrough is effectively creating an output port
/// that feeds through the input to the Diagram and that can now be connected to
/// the inputs of the inner subsystems to the Diagram.
/// A detailed discussion of the PidController can be found at
/// https://github.com/RobotLocomotion/drake/pull/3132.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to http://drake.mit.edu/cxx_inl.html.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// @ingroup primitive_systems
template <typename T>
class PassThrough : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PassThrough)

  /// Constructs a pass through system (`y = u`).
  /// @param size number of elements in the signal to be processed.
  explicit PassThrough(int size);

  /// Constructs a pass thorough system (`y = u`).
  /// @param model_value A template abstract value.
  explicit PassThrough(const AbstractValue& model_value);

  // TODO(eric.cousineau): Possibly share single port interface with
  // ZeroOrderHold (#6490).

  /// Returns the sole input port.
  const InputPortDescriptor<T>& get_input_port() const {
    return LeafSystem<T>::get_input_port(0);
  }

  // Don't use the indexed get_input_port when calling this system directly.
  void get_input_port(int) = delete;

  /// Returns the sole output port.
  const OutputPort<T>& get_output_port() const {
    return LeafSystem<T>::get_output_port(0);
  }

  // Don't use the indexed get_output_port when calling this system directly.
  void get_output_port(int) = delete;

 protected:
  /// Sets the output port to equal the input port.
  void DoCalcVectorOutput(
      const Context<T>& context,
      BasicVector<T>* output) const override;

  // Same as `DoCalcVectorOutput`, but for abstract values.
  void DoCalcAbstractOutput(
      const Context<T>& context,
      AbstractValue* output) const;

  // Override feedthrough detection to avoid the need for `DoToSymbolic()`.
  bool DoHasDirectFeedthrough(const SystemSymbolicInspector* sparsity,
                              int input_port, int output_port) const override;

  /// Returns an PassThrough<symbolic::Expression> with the same dimensions as
  /// this PassThrough.
  PassThrough<symbolic::Expression>* DoToSymbolic() const override;

 private:
  bool is_abstract() const { return abstract_model_value_ != nullptr; }

  const std::unique_ptr<const AbstractValue> abstract_model_value_;
};

}  // namespace systems
}  // namespace drake
