#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/sensors/accelerometer.h"
#include "drake/systems/sensors/test/accelerometer_test/accelerometer_test_logger.h"

namespace drake {
namespace systems {
namespace sensors {

/// Implements the Diagram shown below, which demonstrates how an accelerometer
/// can be used.
///
/// <pre>
///       ------------------------
///       | ConstantVectorSource |
///       ------------------------
///                 |
///                 | < vector of zeros > (actuator torque commands)
///                 |
///                 V
/// -----------------------------------  x_dot
/// |        RigidBodyPlant           |------------------
/// -----------------------------------                 |
///                 |                                   |
///                 |                                   |
///     ------------| x                                 |
///     |           |                                   |
///     |           |                                   |
///     |           |                                   |
///     |           |                                   |
///     |           |                                   |
///     |           |                                   |
///     |           |                                   |
///     |           V                                   |
///     |   -----------------                           |
///     |   | Accelerometer | <-------------------------|
///     |   -----------------                           |
///     |           |                                   |
///     V           V                                   |
/// -------------------------------------               |
/// |      AccelerometerTestLogger      | <--------------
/// -------------------------------------
/// </pre>
///
/// The `ConstantVectorSource` outputs a vector of zeros, which effectively
/// commands the model's actuators to remain passive.
///
/// The RigidBodyPlant contains a model of a pendulum, see
/// drake/examples/pendulum/Pendulum.urdf.
///
/// The Accelerometer is attached to the pendulum's swing arm at (0, 0, -0.5)
/// in the pendlum swing arm's frame.
///
/// A logger is used to store both the plant's state and accelerometer readings.

class AccelerometerExampleDiagram : public Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AccelerometerExampleDiagram);

  AccelerometerExampleDiagram();

  /// Initializes this diagram.
  ///
  /// @param[in] visualizer The visualizer to include with the diagram. This can
  /// be nullptr.
  void Initialize(std::unique_ptr<DrakeVisualizer> visualizer = nullptr);

  /// @name Mutators
  //@{

  /// Sets the initial state of the pendulum.
  ///
  /// @param[in] context This Diagram's context.
  /// @param[in] q The initial position of the pendulum's pivot joint.
  /// @param[in] v The initial velocity of the pendulum's pivot joint.
  void SetInitialState(Context<double>* context, double q, double v);
  //@}

  /// @name Accessors
  //@{
  const Accelerometer& get_accelerometer() const { return *accelerometer_; }

  const RigidBodyTree<double>& get_tree() const { return *tree_; }

  RigidBodyPlant<double>* get_plant() const {
    return plant_;
  }

  int get_model_instance_id() const { return model_instance_id_; }

  const RigidBodyFrame<double>& get_sensor_frame() const {
    return *sensor_frame_.get();
  }

  const AccelerometerTestLogger& get_logger() const { return *logger_; }
  AccelerometerTestLogger* get_mutable_logger() { return logger_; }
  //@}

 private:
  DiagramBuilder<double> builder_;
  RigidBodyTree<double>* tree_{nullptr};
  int model_instance_id_{};
  std::shared_ptr<RigidBodyFrame<double>> sensor_frame_;
  RigidBodyPlant<double>* plant_{nullptr};
  Accelerometer* accelerometer_{nullptr};
  AccelerometerTestLogger* logger_{nullptr};
  DrakeVisualizer* visualizer_{nullptr};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
