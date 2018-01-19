#include <vector>
#include <string>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/bindings/pydrake/util/cpp_template_pybind.h"
#include "drake/bindings/pydrake/util/type_pack.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/pixel_types.h"

namespace py = pybind11;

using std::string;
using std::vector;

namespace drake {

namespace pydrake {

template <typename T, T Value>
using constant = std::integral_constant<T, Value>;

template <typename T, T ... Values>
using constant_pack = type_pack<type_pack<constant<T, Values>>...>;

namespace {

using Eigen::Map;
using Eigen::Ref;

// TODO(eric.cousineau): Place in `pydrake_pybind.h`.
template <typename T>
py::object ToArray(T* ptr, int size, py::tuple shape) {
  // Create flat array, to be reshaped.
  using Vector = VectorX<T>;
  Map<Vector> data(ptr, size);
  return py::cast(Ref<Vector>(data)).attr("reshape")(shape);
}

// `const` variant.
template <typename T>
py::object ToArray(const T* ptr, int size, py::tuple shape) {
  // Create flat array, to be reshaped.
  using Vector = const VectorX<T>;
  Map<Vector> data(ptr, size);
  return py::cast(Ref<Vector>(data)).attr("reshape")(shape);
}

}  // namespace
}  // namespace pydrake
}  // namespace drake

PYBIND11_MODULE(sensors, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::pydrake;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::sensors;

  auto py_iref = py::return_value_policy::reference_internal;

  m.doc() = "Bindings for the sensors portion of the Systems framework.";

  // Expose only types that are used.
  py::enum_<PixelFormat>(m, "PixelFormat")
    .value("kRgba", PixelFormat::kRgba)
    .value("kDepth", PixelFormat::kDepth)
    .value("kLabel", PixelFormat::kLabel);

  {
    // Expose image types and their traits.
    py::enum_<PixelType> pixel_type(m, "PixelType");
    vector<string> enum_names = {
        "kRgba8U",
        "kDepth32F",
        "kLabel16I",
    };

    using ParamList = constant_pack<PixelType,
        PixelType::kRgba8U,
        PixelType::kDepth32F,
        PixelType::kLabel16I>;

    // Simple constexpr for-loop.
    int i = 0;
    auto instantiation_visitor = [&](auto param) {
      // Add definition to enum.
      using Param = decltype(param);
      static_assert(Param::size == 1, "Should have scalar type_pack");
      constexpr PixelType Value = Param::template type_at<0>::value;
      pixel_type.value(enum_names[i].c_str(), Value);

      py::tuple py_param = GetPyParam(param);
      using ImageT = Image<Value>;
      using ImageTraitsT = ImageTraits<Value>;
      using T = typename ImageTraitsT::ChannelType;

      // Add traits.
      py::class_<ImageTraitsT> traits(
          m, TemporaryClassName<ImageTraitsT>().c_str());
      traits.attr("ChannelType") = GetPyParam<T>()[0];
      traits.attr("kNumChannels") = int{ImageTraitsT::kNumChannels};
      traits.attr("kPixelFormat") = PixelFormat{ImageTraitsT::kPixelFormat};
      AddTemplateClass(m, "ImageTraits", traits, py_param);

      // Shape for use with NumPy, OpenCV, etc. Using same shape as what is
      // present in `show_images.py`.
      auto get_shape = [](const ImageT* self) {
        return py::make_tuple(
            self->height(), self->width(), int{ImageTraitsT::kNumChannels});
      };

      py::class_<ImageT> image(m, TemporaryClassName<ImageT>().c_str());
      image
          .def(py::init<int, int>())
          .def(py::init<int, int, T>())
          .def("width", &ImageT::width)
          .def("height", &ImageT::height)
          .def("size", &ImageT::size)
          .def("resize", &ImageT::resize)
          .def("at", [](const ImageT* self, int x, int y) {
                return *self->at(x, y);
              })
          .def("set", [](ImageT* self, int x, int y, T value) {
                *self->at(x, y) = value;
              })
          .def("shape", get_shape)
          .def("array", [=](const ImageT* self) {
                return ToArray(self->at(0, 0), self->size(), get_shape(self));
              }, py_iref)
          .def("mutable_array", [=](ImageT* self) {
                return ToArray(self->at(0, 0), self->size(), get_shape(self));
              }, py_iref);
      // Constants.
      image.attr("Traits") = traits;
      // - Do not duplicate aliases (e.g. `kNumChannels`) for now.
      AddTemplateClass(m, "Image", image, py_param);
      // Add type alias for instantiation.
      m.attr(("Image" + enum_names[i].substr(1)).c_str()) = image;
      // Ensure that iterate.
      ++i;
    };
    type_visit(instantiation_visitor, ParamList{});
  }

  // Constants.
  py::class_<InvalidDepth> invalid_depth(m, "InvalidDepth");
  invalid_depth.attr("kTooFar") = InvalidDepth::kTooFar;
  invalid_depth.attr("kTooClose") = InvalidDepth::kTooClose;

  py::class_<Label> label(m, "Label");
  label.attr("kTooFar") = Label::kNoBody;
  label.attr("kTooClose") = Label::kFlatTerrain;
}
