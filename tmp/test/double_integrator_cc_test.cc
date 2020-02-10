// #include "drake/systems/primitives/vector_system.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace systems {
namespace {

using Eigen::MatrixXd;
using drake::systems::controllers::LinearQuadraticRegulator;

int DoMain() {
  MatrixXd A(2, 2);
  A << 0, 1, 0, 0;
  MatrixXd B(2, 1);
  B << 0, 1;
  MatrixXd C = MatrixXd::Identity(2, 2);
  MatrixXd D = MatrixXd::Zero(2, 1);

  DiagramBuilder<double> builder;

  auto* plant = builder.AddSystem(
      std::make_unique<LinearSystem<double>>(A, B, C, D));
  MatrixXd Q = MatrixXd::Identity(2, 2);
  MatrixXd R = MatrixXd::Identity(1, 1);
  auto* controller = builder.AddSystem(
      LinearQuadraticRegulator(*plant, Q, R));

  builder.Connect(controller->get_output_port(), plant->get_input_port());
  builder.Connect(plant->get_output_port(), controller->get_input_port());
  auto diagram = builder.Build();

  Simulator<double> simulator(*diagram);
  simulator.Initialize();
  MatrixXd x0(2, 1);
  x0 << 0, 0;
  simulator.get_mutable_context().SetContinuousState(x0);
  simulator.AdvanceTo(1.);

  std::cout << "Done\n";
  return 0;
}

}  // namespace
}  // namespace systems
}  // namespace drake

int main(int, char**) {
  return drake::systems::DoMain();
}
