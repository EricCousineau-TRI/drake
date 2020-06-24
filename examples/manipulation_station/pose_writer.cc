#include "drake/examples/manipulation_station/pose_writer.h"

namespace drake {
namespace multibody {

PoseWriter::PoseWriter(
    const MultibodyPlant<double>* plant,
    const std::vector<const Frame<double>*>& frames,
    const std::map<std::string, std::string>& frame_name_to_output_name,
    const std::string& file_name, double period, double t_offset)
    : plant_(*plant),
      context_(plant_.CreateDefaultContext()),
      frames_(frames),
      frame_name_to_output_name_(frame_name_to_output_name),
      out_(std::ofstream(file_name)) {
  this->DeclareVectorInputPort(
      "state_input",
      systems::BasicVector<double>(plant_.num_multibody_states()));
  this->DeclarePeriodicPublishEvent(period, t_offset, &PoseWriter::WritePose);
}

void PoseWriter::WritePose(const systems::Context<double>& context) const {
  const systems::BasicVector<double>* x = this->EvalVectorInput(context, 0);
  plant_.SetPositionsAndVelocities(context_.get(), x->get_value());

  auto write = [](const math::RigidTransform<double>& pose,
                  const std::string& name, std::ofstream* out) {
    const auto quat = pose.rotation().ToQuaternion();
    const auto& trans = pose.translation();
    (*out) << "  " << name << ":\n";
    (*out) << "    "
           << "quaternion:\n";
    (*out) << "      "
           << "w: " << fmt::format("{:.12f}", quat.w()) << "\n";
    (*out) << "      "
           << "x: " << fmt::format("{:.12f}", quat.x()) << "\n";
    (*out) << "      "
           << "y: " << fmt::format("{:.12f}", quat.y()) << "\n";
    (*out) << "      "
           << "z: " << fmt::format("{:.12f}", quat.z()) << "\n";
    (*out) << "    "
           << "translation:\n";
    (*out) << "      "
           << "x: " << fmt::format("{:.12f}", trans.x()) << "\n";
    (*out) << "      "
           << "y: " << fmt::format("{:.12f}", trans.y()) << "\n";
    (*out) << "      "
           << "z: " << fmt::format("{:.12f}", trans.z()) << "\n";
  };

  const std::string rgb_img_name =
      fmt::format("{count:06}_rgb.png", fmt::arg("count", ctr_));
  const std::string depth_img_name =
      fmt::format("{count:06}_depth.png", fmt::arg("count", ctr_));
  (out_) << ctr_ << ":\n";

  for (const Frame<double>* frame : frames_) {
    auto X_WF =
        plant_.CalcRelativeTransform(*context_, plant_.world_frame(), *frame);
    write(X_WF, frame_name_to_output_name_.at(frame->name()), &out_);
  }

  (out_) << "  "
         << "depth_image_filename: " << depth_img_name << "\n";
  (out_) << "  "
         << "rgb_image_filename: " << rgb_img_name << "\n";
  (out_) << "  "
         << "timestamp: " << static_cast<uint64_t>(context.get_time() * 1e6)
         << "\n";

  out_.flush();
  ctr_++;
}

VectorWriter::VectorWriter(const std::string& file_name,
                           const std::string& name, int size, double period,
                           double t_offset)
    : name_(name), out_(std::ofstream(file_name)) {
  this->DeclareVectorInputPort("input", systems::BasicVector<double>(size));
  this->DeclarePeriodicPublishEvent(period, t_offset,
                                    &VectorWriter::WriteVector);
}

void VectorWriter::WriteVector(const systems::Context<double>& context) const {
  const systems::BasicVector<double>* x = this->EvalVectorInput(context, 0);
  out_ << ctr_ << ":\n";
  out_ << "  " << name_ << ": [" << x->get_value()[0];
  for (int i = 1; i < x->get_value().size(); i++) {
    out_ << ", " << x->get_value()[i];
  }
  out_ << "]\n";
  out_ << "  "
       << "timestamp: " << static_cast<uint64_t>(context.get_time() * 1e6)
       << "\n";
  out_.flush();
  ctr_++;
}

}  // namespace multibody
}  // namespace drake
