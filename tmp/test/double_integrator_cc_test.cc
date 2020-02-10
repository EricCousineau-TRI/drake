#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"

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

  auto context = diagram->CreateDefaultContext();
  MatrixXd x0(2, 1);
  x0 << 0, 0;
  context->SetContinuousState(x0);
  plant->get_output_port().Eval(plant->GetMyContextFromRoot(*context));

  std::cout << "Done\n";
  return 0;
}

}  // namespace
}  // namespace systems
}  // namespace drake

int main(int, char**) {
  return drake::systems::DoMain();
}
