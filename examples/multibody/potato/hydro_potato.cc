#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_double(rez_hint, 0.12,
              "The resolution hint to use on the peeler box; "
              "must be less than 0.02 to register contact");

namespace drake {
namespace examples {
namespace multibody {
namespace potato {

using drake::geometry::ConnectDrakeVisualizer;
using drake::geometry::ProximityProperties;
using drake::multibody::ConnectContactResultsToDrakeVisualizer;
using drake::multibody::ContactModel;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RigidBody;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;

int do_main() {
  DiagramBuilder<double> builder;

  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<MultibodyPlant<double>>(0.0));

  Parser parser(&plant);
  parser.AddModelFromFile(
      FindResourceOrThrow("drake/examples/multibody/potato/models/peeler.sdf"));
  parser.AddModelFromFile(FindResourceOrThrow(
      "drake/examples/multibody/potato/models/texture_123-87.sdf"));

  const RigidBody<double>& peeler = plant.GetRigidBodyByName("peeler");
  const auto frame_id = plant.GetBodyFrameIdOrThrow(peeler.index());
  const auto& inspector = scene_graph.model_inspector();
  auto geometries =
      inspector.GetGeometries(frame_id, geometry::Role::kProximity);
  DRAKE_DEMAND(geometries.size() == 1);
  const ProximityProperties* old_props =
      inspector.GetProximityProperties(geometries[0]);
  DRAKE_DEMAND(old_props != nullptr);
  if (FLAGS_rez_hint !=
      old_props->GetProperty<double>("hydroelastic", "resolution_hint")) {
    ProximityProperties new_props(*old_props);
    new_props.UpdateProperty("hydroelastic", "resolution_hint", FLAGS_rez_hint);
    scene_graph.AssignRole(*plant.get_source_id(), geometries[0], new_props,
                           geometry::RoleAssign::kReplace);
  }

  plant.mutable_gravity_field().set_gravity_vector({0, 0, 0});
  plant.set_contact_model(ContactModel::kHydroelasticsOnly);
  plant.Finalize();

  ConnectDrakeVisualizer(&builder, scene_graph);
  ConnectContactResultsToDrakeVisualizer(&builder, plant);

  auto diagram = builder.Build();

  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  const math::RigidTransformd X_WP(Vector3<double>{0, 0, 0.0225});
  plant.SetFreeBodyPose(&plant_context, peeler, X_WP);

  Simulator<double> simulator(*diagram, std::move(diagram_context));

  /* Note: these start in contact. If there is a contact force, it will
   *immediately* apply an impulse that will push them apart. They separate
   quickly (less than 0.2 s). However, the contact publishing happens at 60 Hz,
   so we'll get contact shown at 0 and 0.166667 seconds.  */
  simulator.set_target_realtime_rate(0.005);
  /* Make sure we draw at t = 0, 0.016667, and 0.0333333.  */
  simulator.AdvanceTo(0.034);

  return 0;
}
}  // namespace potato
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::potato::do_main();
}
