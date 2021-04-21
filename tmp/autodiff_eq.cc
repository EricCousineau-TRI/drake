#include <iostream>

#include <Eigen/Dense>

#include "drake/common/autodiff.h"

using Eigen::Vector2d;

namespace drake {
namespace {

void DoMain() {
  AutoDiffXd a(1.0, Vector2d(1.0, 2.0));
  AutoDiffXd b(1.0, Vector2d(3.0, 4.0));

  std::cout << ((a == b) ? "True" : "False") << std::endl;
}

}
}

int main(int, char**) {
  drake::DoMain();
  return 0;
}

/**
Output:

$ bazel run //tmp:autodiff_eq
...
True
*/
