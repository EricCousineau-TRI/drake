#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>

#include "drake/multibody/plant/multibody_plant.h"

namespace py = pybind11;

namespace repro {

using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::geometry::HalfSpace;

template <typename T>
void addFlatTerrain(MultibodyPlant<T>* plant, SceneGraph<T>* scene_graph,
                    double mu_static, double mu_kinetic,
                    Eigen::Vector3d normal_W) {
  if (!plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }

  Eigen::Vector3d point_W(0, 0, 0);
  drake::multibody::CoulombFriction<T> friction(mu_static, mu_kinetic);

  // A half-space for the ground geometry.
  const drake::math::RigidTransformd X_WG(
      HalfSpace::MakePose(normal_W, point_W));

  plant->RegisterCollisionGeometry(plant->world_body(), X_WG, HalfSpace(),
                                   "collision", friction);

  // Add visual for the ground.
  plant->RegisterVisualGeometry(plant->world_body(), X_WG, HalfSpace(),
                                "visual");
}

PYBIND11_MODULE(cc, m) {
  py::module::import("pydrake.all");

  m.def("addFlatTerrain",
      &addFlatTerrain<double>, py::arg("plant"),
      py::arg("scene_graph"), py::arg("mu_static"), py::arg("mu_kinetic"),
      py::arg("normal_W") = Eigen::Vector3d(0, 0, 1));
}

}  // namespace repro
