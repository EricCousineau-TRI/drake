#include "fmt/format.h"
#include "robotlocomotion/quaternion_t.hpp"

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace tmp {

using fmt::print;
using namespace drake::lcm;
using namespace drake::systems::lcm;
using namespace drake::systems;
using namespace drake;

class PrintTime : public LeafSystem<double> {
 public:
  using Base = LeafSystem<double>;
  using Base::DeclarePeriodicPublish;
 protected:
  void DoPublish(
      const Context<double>& context,
      const std::vector<const PublishEvent<double>*>&) const override {
    print("t: {}\n", context.get_time());
  }
};

int DoMain() {
  DrakeLcm lcm("memq://");
  using lcm_type = robotlocomotion::quaternion_t;
  const std::string channel = "JUNK";
  const double dt_sec = 1.;

  DiagramBuilder<double> builder;
  builder.AddSystem<PrintTime>()->DeclarePeriodicPublish(dt_sec);
  builder.AddSystem<LcmInterfaceSystem>(&lcm);
  builder.AddSystem(LcmSubscriberSystem::Make<lcm_type>(channel, &lcm));
  auto diagram = builder.Build();

  for (bool do_publish : {false, true}) {
    print("--- do_publish: {} ---\n", do_publish);
    Simulator<double> simulator(*diagram);
    if (do_publish) {
      Publish(&lcm, channel, lcm_type());
    }
    print("Initialize\n");
    simulator.Initialize();
    print("AdvanceTo\n");
    simulator.AdvanceTo(dt_sec);
    print("\n");
  }
  return 0;
}

}  // namespace tmp

int main() {
  return tmp::DoMain();
}

/*
--- do_publish: false ---
Initialize
t: 0.0
AdvanceTo
t: 1.0

--- do_publish: true ---
Initialize
AdvanceTo
t: 1.0
*/
