#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

template <typename Data, typename T = double>
class AbstractZOH : public LeafSystem<T> {
 public:
  typedef std::function<void(double time, const Data& value)> OnUpdate;

  AbstractZOH(double period_sec, bool use_autoinit = false)
      : AbstractZOH(Data(), period_sec, 0., use_autoinit) {}

  AbstractZOH(const Data& ic, double period_sec, double offset_sec = 0.,
              bool use_autoinit = false)
      : use_autoinit_(use_autoinit) {
    this->DeclareAbstractInputPort();
    // TODO(eric.cousineau): Is there a way to not care about the default
    // constructor, and just inherit this from an upstream system given
    // type erasure?
    this->DeclareAbstractState(std::make_unique<Value<Data>>(ic));
    this->DeclareAbstractOutputPort(ic, &AbstractZOH::CalcOutput);
    this->DeclarePeriodicUnrestrictedUpdate(period_sec, offset_sec);
  }

  void ConnectOnUpdate(const OnUpdate& on_update) {
    DRAKE_ASSERT(!on_update_);
    on_update_ = on_update;
  }

 protected:
  void DoCalcUnrestrictedUpdate(const Context<T>& context,
                                State<T>* state) const override {
    const Data& input_value =
        this->EvalAbstractInput(context, 0)->template GetValue<Data>();
    Data& stored_value =
        state->template get_mutable_abstract_state<Data>(0);
    stored_value = input_value;
    if (on_update_) {
      on_update_(context.get_time(), input_value);
    }
  }

  void SetDefaultState(const Context<T>& context,
                       State<T>* state) const override {
    if (use_autoinit_) {
      // Update initial values with the ICs from upstream blocks.
      this->DoCalcUnrestrictedUpdate(context, state);
    } else {
      LeafSystem<T>::SetDefaultState(context, state);
    }
  }

  void CalcOutput(const Context<T>& context,
                    Data* poutput) const {
    const Data& stored_value =
        context.template get_abstract_state<Data>(0);
    *poutput = stored_value;
  }
 private:
  OnUpdate on_update_;
  bool use_autoinit_{};
};

//template <typename T, typename... Extra>
//std::unique_ptr<AbstractZOH<T>> MakeAbstractZOH(const T& ic, Extra&&... extra) {
//  return std::make_unique<AbstractZOH<T>>(ic, std::forward<Extra>(extra)...);
//}

//template <typename Visitor>
//void pack_visit(Visitor&& visitor) {}

//// Visit a set of parameter pack template arguments, using Visitor::run<T>
//template <typename Visitor, typename T, typename... Ts>
//void pack_visit(Visitor&& visitor) {
//  visitor.template run<T>();
//  pack_visit<Visitor, Ts...>(std::forward<Visitor>(visitor));
//};

//// To infer caller type
//template <typename... Ts>
//struct pack_visitor {
//  template <typename Visitor>
//  static void run(Visitor&& visitor) {
//    pack_visit<Visitor, Ts...>(std::forward<Visitor>(visitor));
//  }
//};

//template <typename Visitor>
//void param_visit(Visitor&& visitor) {}

//// Visit a set of parameter, using Visitor::run<T>(T&& t)
//template <typename Visitor, typename T, typename... Ts>
//void param_visit(Visitor&& visitor, T&& arg, Ts&&... args) {
//  visitor(std::forward<T>(arg));
//  pack_visit<Visitor, Ts...>(std::forward<Visitor>(visitor),
//                             std::forward<Ts>(args)...);
//};

/**
 * Stack a group of SISO systems.
 * Example:
 *   auto* stack = StackSystems(true, true, {
 *     new AbstractZOH(...),
 *     new AbstractZOH(...)});
 */
template <typename T>
std::unique_ptr<Diagram<T>> StackSystems(
    bool export_input, bool export_output,
    const std::vector<LeafSystem<T>*>& systems) {
  DRAKE_ASSERT(export_input || export_output);
  DiagramBuilder<T> builder;
  for (LeafSystem<T>* system : systems) {
    builder.AddSystem(std::unique_ptr<LeafSystem<T>>(system));
    if (export_input) {
      builder.ExportInput(system->get_input_port(0));
    }
    if (export_output) {
      builder.ExportOutput(system->get_output_port(0));
    }
  }
  return builder.Build();
}

}  // namespace systems
}  // namespace drake
