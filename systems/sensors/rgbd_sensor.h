#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/render/camera_properties.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_vector.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

/** A sensor that provides RGB, depth, and label images using the geometry in
 the geometry::SceneGraph.

 @system{RgbdSensor,
    @input_port{geometry_query},
    @output_port{color_image}
    @output_port{depth_image_32f}
    @output_port{depth_image_16u}
    @output_port{label_image}
    @output_port{X_WB}
 }

 The following text uses terminology and camera frame conventions from
 CameraInfo. Please review its documentation.

 This class uses the following frames:

   - W - world frame
   - C - color camera frame, used for both color and label cameras to guarantee
     perfect registration between color and label images.
   - D - depth camera frame
   - B - sensor body frame, approximately, the frame of the "physical" sensor
     that contains the color, depth, and label cameras. The contained cameras
     are rigidly fixed to B and X_WB is what is used to pose the sensor in the
     world (or, alternatively, X_PB where P is some parent frame for which X_WP
     is known).

 By default, frames B, C, and D are coincident and aligned. These can be
 changed after construction by modifying `X_BC` and `X_BD`. Frames C and D are
 always rigidly affixed to the sensor body frame B.

 <!-- TODO(gizatt): The setters for modifying the sensor poses create a
 vulnerability that allows users to modify internal system state during
 simulation via a non-intended path. See PR#10491 for discussion;
 solutions could include enshrining these poses as proper parameters
 or accepting these poses during construction.-->

 Output port image formats:

   - color_image: Four channels, each channel uint8_t, in the following order:
     red, green, blue, and alpha.

   - depth_image_32f: One channel, float, representing the Z value in
     `D` in *meters*. The values 0 and infinity are reserved for out-of-range
     depth returns (too close or too far, respectively, as defined by
     DepthCameraProperties).

   - depth_image_16u: One channel, uint16_t, representing the Z value in
     `D` in *millimeters*. The values 0 and 65535 are reserved for out-of-range
     depth returns (too close or too far, respectively, as defined by
     DepthCameraProperties). Additionally, 65535 will also be returned if the
     depth measurement exceeds the representation range of uint16_t. Thus, the
     maximum valid depth return is 65534mm.

   - label_image: One channel, int16_t, whose value is a unique RenderLabel
     value aligned with the color camera frame. See RenderLabel for discussion
     of interpreting rendered labels.

 @note These depth sensor measurements differ from those of range data used by
 laser range finders (like DepthSensor), where the depth value represents the
 distance from the sensor origin to the object's surface.

 @ingroup sensor_systems  */
class RgbdSensor final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdSensor)

  // TODO(SeanCurtis-TRI): Seriously consider a OpenGL-style position-target-up
  //  constructor for RgbdSensor -- it can work for either stationary or
  //  generally affixed sensor as long as the position, target, and up vectors
  //  are all defined w.r.t. the parent frame.

  /** Constructs an %RgbdSensor whose frame `B` is rigidly affixed to the frame
   P, indicated by `frame_id`, and with the given camera properties. The camera
   will move as frame P moves. For a stationary camera, use the frame id from
   SceneGraph::world_frame_id().

   @param name           The name of the %RgbdSensor. This can be any value, but
                         typically should be unique.
   @param frame_id       The identifier of a parent frame `P` in
                         geometry::SceneGraph to which this camera is rigidly
                         affixed with pose `X_PB`.
   @param X_PB           The pose of the camera `B` frame relative to the parent
                         frame `P`.
   @param properties     The properties which define this camera's intrinsics.
   @param show_window    A flag for showing a visible window. If this is false,
                         off-screen rendering is executed. The default is false.
   */
  RgbdSensor(std::string name, geometry::FrameId parent_frame,
             const math::RigidTransformd& X_PB,
             const geometry::render::DepthCameraProperties& properties,
             bool show_window = false);

  ~RgbdSensor() = default;

  /** Returns the color sensor's info.  */
  const CameraInfo& color_camera_info() const { return color_camera_info_; }

  /** Returns the depth sensor's info.  */
  const CameraInfo& depth_camera_info() const { return depth_camera_info_; }

  /** Returns `X_BC`.  */
  const math::RigidTransformd& X_BC() const {
    return X_BC_;
  }

  /** Sets `X_BC`.  */
  void set_X_BC(const math::RigidTransformd& X_BC) {
    X_BC_ = X_BC;
  }

  /** Returns `X_BD`.  */
  const math::RigidTransformd& X_BD() const {
    return X_BD_;
  }

  /** Sets `X_BD`.  */
  void set_X_BD(const math::RigidTransformd& X_BD) {
    X_BD_ = X_BD;
  }

  /** Returns the id of the frame to which the base is affixed.  */
  geometry::FrameId parent_frame_id() const { return parent_frame_; }

  /** Returns the geometry::QueryObject<double>-valued input port.  */
  const InputPort<double>& query_object_input_port() const;

  /** Returns the abstract-valued output port that contains an ImageRgba8U.  */
  const OutputPort<double>& color_image_output_port() const;

  /** Returns the abstract-valued output port that contains an ImageDepth32F.
   */
  const OutputPort<double>& depth_image_32F_output_port() const;

  /** Returns the abstract-valued output port that contains an ImageDepth16U.
   */
  const OutputPort<double>& depth_image_16U_output_port() const;

  /** Returns the abstract-valued output port that contains an ImageLabel16I.
   */
  const OutputPort<double>& label_image_output_port() const;

  /** Returns the abstract-valued output port that contains a RigidTransform
    for `X_WB`.
   */
  const OutputPort<double>& X_WB_output_port() const;

 private:
  friend class RgbdSensorTester;

  // The calculator methods for the four output ports.
  void CalcColorImage(const Context<double>& context,
                      ImageRgba8U* color_image) const;
  void CalcDepthImage32F(const Context<double>& context,
                         ImageDepth32F* depth_image) const;
  void CalcDepthImage16U(const Context<double>& context,
                         ImageDepth16U* depth_image) const;
  void CalcLabelImage(const Context<double>& context,
                      ImageLabel16I* label_image) const;
  void CalcX_WB(const Context<double>& context,
                rendering::PoseVector<double>* pose_vector) const;

  // Convert a single channel, float depth image (with depths in meters) to a
  // single channel, unsigned uint16_t depth image (with depths in millimeters).
  static void ConvertDepth32FTo16U(const ImageDepth32F& d32,
                                   ImageDepth16U* d16);

  // Extract the query object from the given context (via the appropriate input
  // port.
  const geometry::QueryObject<double>& get_query_object(
      const Context<double>& context) const;

  const InputPort<double>* query_object_input_port_{};
  const OutputPort<double>* color_image_port_{};
  const OutputPort<double>* depth_image_32F_port_{};
  const OutputPort<double>* depth_image_16U_port_{};
  const OutputPort<double>* label_image_port_{};
  const OutputPort<double>* sensor_base_pose_port_{};

  // The identifier for the parent frame `P`.
  const geometry::FrameId parent_frame_;

  // If true, a window will be shown for the camera.
  const bool show_window_;
  const CameraInfo color_camera_info_;
  const CameraInfo depth_camera_info_;
  const geometry::render::DepthCameraProperties properties_;
  // The position of the camera's B frame relative to its parent frame P.
  const math::RigidTransformd X_PB_;

  math::RigidTransformd X_BC_;

  math::RigidTransformd X_BD_{X_BC_};
};

/**
 Wraps a continuous %RgbdSensor with a zero-order hold to create a discrete
 sensor.

 @system{%RgbdSensorDiscrete,
    @input_port{geometry_query},
    @output_port{color_image}
    @output_port{depth_image_32f}
    @output_port{depth_image_16u}
    @output_port{label_image}
    @output_port{X_WB}
 }
 */
class RgbdSensorDiscrete final : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdSensorDiscrete);

  static constexpr double kDefaultPeriod = 1. / 30;

  /** Constructs a diagram containing a (non-registered) RgbdSensor that will
   update at a given rate.
   @param sensor               The continuous sensor used to generate periodic
                               images.
   @param period               Update period (sec).
   @param render_label_image   If true, renders label image (which requires
                               additional overhead). If false,
                               `label_image_output_port` will raise an error if
                               called.  */
  RgbdSensorDiscrete(std::unique_ptr<RgbdSensor> sensor,
                     double period = kDefaultPeriod,
                     bool render_label_image = true);

  /** Returns reference to RgbdSensor instance.  */
  const RgbdSensor& sensor() const { return *camera_; }

  /** Returns mutable reference to RgbdSensor instance.  */
  RgbdSensor& mutable_sensor() { return *camera_; }

  /** Returns update period for discrete camera.  */
  double period() const { return period_; }

  /** @see RgbdSensor::query_object_input_port().  */
  const InputPort<double>& query_object_input_port() const {
    return get_input_port(query_object_port_);
  }

  /** @see RgbdSensor::color_image_output_port().  */
  const systems::OutputPort<double>& color_image_output_port() const {
    return get_output_port(output_port_color_image_);
  }

  /** @see RgbdSensor::depth_image_32F_output_port().  */
  const systems::OutputPort<double>& depth_image_32F_output_port() const {
    return get_output_port(output_port_depth_image_32F_);
  }

  /** @see RgbdSensor::depth_image_16U_output_port().  */
  const systems::OutputPort<double>& depth_image_16U_output_port() const {
    return get_output_port(output_port_depth_image_16U_);
  }

  /** @see RgbdSensor::label_image_output_port().  */
  const systems::OutputPort<double>& label_image_output_port() const {
    return get_output_port(output_port_label_image_);
  }

  /** @see RgbdSensor::base_pose_output_port().  */
  const systems::OutputPort<double>& base_pose_output_port() const {
    return get_output_port(output_port_pose_);
  }

 private:
  RgbdSensor* const camera_{};
  const double period_{};

  int query_object_port_{-1};
  int output_port_color_image_{-1};
  int output_port_depth_image_32F_{-1};
  int output_port_depth_image_16U_{-1};
  int output_port_label_image_{-1};
  int output_port_pose_{-1};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
