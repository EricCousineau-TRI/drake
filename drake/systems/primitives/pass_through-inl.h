#pragma once

/// @file
/// Template method implementations for pass_through.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/common/unused.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace systems {

// TODO(amcastro-tri): remove the size parameter from the constructor once
// #3109 supporting automatic sizes is resolved.
template <typename T>
PassThrough<T>::PassThrough(int size) : VectorSystem<T>(size, size) { }

template <typename T>
PassThrough<T>::PassThrough(const AbstractValue& model_value)
    : VectorSystem<T>(size, size) { }

template <typename T>
void PassThrough<T>::DoCalcVectorOutput(
      const Context<T>& context,
      BasicVector<T>* output) const {
  DRAKE_ASSERT(!is_abstract());
  const BasicVector<T>& input = *this->EvalVectorInput(context, 0);
  *output = input;
}

template <typename T>
void PassThrough<T>::DoCalcAbstractOutput(const Context<T>& context,
                                          AbstractValue* output) const {
  DRAKE_ASSERT(is_abstract());
  const AbstractValue& input =
      *this->EvalAbstractInput(context, 0);
  output->SetFrom(input);
}

template <typename T>
bool PassThrough<T>::DoHasDirectFeedthrough(
    const SystemSymbolicInspector*, int input_port, int output_port) const {
  DRAKE_DEMAND(input_port == 0);
  DRAKE_DEMAND(output_port == 0);
  // By definition, a pass-through will have direct feedthrough, as the
  // output only depends on the input.
  return true;
}

template <typename T>
PassThrough<symbolic::Expression>* PassThrough<T>::DoToSymbolic() const {
  if (!is_abstract()) {
    return new PassThrough<symbolic::Expression>(this->get_input_port().size());
  else {
    // Return `nullptr` to enable control to reach `DoHasDirectFeedthrough`.
    return nullptr;
  }
}

}  // namespace systems
}  // namespace drake
