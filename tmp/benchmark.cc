#include <chrono>

#include "drake/math/autodiff.h"
#include "drake/solvers/cost.h"

using namespace drake;
using namespace drake::solvers;

int main() {

  LinearCost cost(Eigen::VectorXd::Random(20));
  auto x = math::initializeAutoDiff(Eigen::VectorXd::Random(20));
  AutoDiffVecXd y;
  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 100000; ++i) {
    cost.Eval(x, &y);
  }
  auto end = std::chrono::steady_clock::now();
  std::cout <<
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
      <<"\n";
  return 0;
}
