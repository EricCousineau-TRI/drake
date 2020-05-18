#include "drake/geometry/scene_graph_inspector.h"

namespace drake {
namespace geometry {

namespace {

template <typename T>
class GeometryInstanceReifier : public ShapeReifier {
 public:
  GeometryInstanceReifier(
      const SceneGraphInspector<T>* inspector, GeometryId id)
      : inspector_(inspector), id_(id) {}

  using ShapeReifier::ImplementGeometry;
  void ImplementGeometry(const Sphere& shape, void*) final { Run(shape); }
  void ImplementGeometry(const Cylinder& shape, void*) final { Run(shape); }
  void ImplementGeometry(const HalfSpace& shape, void*) final { Run(shape); }
  void ImplementGeometry(const Box& shape, void*) final { Run(shape); }
  void ImplementGeometry(const Capsule& shape, void*) final { Run(shape); }
  void ImplementGeometry(const Ellipsoid& shape, void*) final { Run(shape); }
  void ImplementGeometry(const Mesh& shape, void*) final { Run(shape); }
  void ImplementGeometry(const Convex& shape, void*) final { Run(shape); }

  std::unique_ptr<GeometryInstance> release_value() {
    DRAKE_DEMAND(!!value_);
    return std::move(value_);
  }

 private:
  template <typename ShapeType>
  void Run(const ShapeType& shape) {
    auto name = inspector_->GetName(id_);
    auto X_PG = inspector_->GetPoseInFrame(id_);
    value_ = std::make_unique<GeometryInstance>(
        X_PG, std::make_unique<ShapeType>(shape), name);
    if (auto* props = inspector_->GetProximityProperties(id_)) {
      value_->set_proximity_properties(*props);
    }
    if (auto* props = inspector_->GetIllustrationProperties(id_)) {
      value_->set_illustration_properties(*props);
    }
    if (auto* props = inspector_->GetPerceptionProperties(id_)) {
      value_->set_perception_properties(*props);
    }
  }

  const SceneGraphInspector<T>* inspector_{};
  GeometryId id_;
  std::unique_ptr<GeometryInstance> value_;
};

}  // namespace

template <typename T>
std::unique_ptr<GeometryInstance>
SceneGraphInspector<T>::CloneGeometryInstance(GeometryId id) const {
  GeometryInstanceReifier reifier(this, id);
  Reify(id, &reifier);
  return reifier.release_value();
}

// Explicitly instantiates on the most common scalar types.
template class SceneGraphInspector<double>;
template class SceneGraphInspector<AutoDiffXd>;

}  // namespace geometry
}  // namespace drake
