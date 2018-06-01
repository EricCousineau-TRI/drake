#include <cstring>

#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace pydrake {

using systems::lcm::SerializerInterface;
using systems::AbstractValue;

class PySerializeInterface : public py::wrapper<SerializerInterface> {
 public:
  using Base = py::wrapper<SerializerInterface>;
  using Base::Base;

  std::unique_ptr<AbstractValue> CreateDefaultValue() const override {
    PYBIND11_OVERLOAD_PURE(
        std::unique_ptr<AbstractValue>, SerializerInterface,
        "CreateDefaultValue");
  }

  void Deserialize(
      const void* message_bytes, int message_length,
      AbstractValue* abstract_value) const override {
    // Capture return.
    auto wrapped = [&]() -> std::unique_ptr<AbstractValue> {
      py::bytes buffer(
          reinterpret_cast<const char*>(message_bytes), message_length);
      PYBIND11_OVERLOAD_INT(
          std::unique_ptr<AbstractValue>, SerializerInterface,
          "Deserialize", buffer);
      py::pybind11_fail("No overload defined!");
    };
    abstract_value->SetFrom(*wrapped());
  }

  void Serialize(const AbstractValue& abstract_value,
                 std::vector<uint8_t>* message_bytes) const override {
    // Capture return.
    auto wrapped = [&]() -> py::bytes {
      PYBIND11_OVERLOAD_INT(
        py::bytes, SerializerInterface, "Serialize", abstract_value);
      // Fail if overload not found.
      py::pybind11_fail("No overload defined!");
    };
    std::string str = wrapped();
    message_bytes->resize(str.size());
    std::memcpy(message_bytes->data(), str.data(), str.size());
  }
};

PYBIND11_MODULE(_lcm_py, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::lcm;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::lcm;

  py::module::import("pydrake.lcm");
  py::module::import("pydrake.systems.framework");

  {
    using Class = SerializerInterface;
    py::class_<Class, PySerializeInterface>(m, "SerializerInterface")
        .def("CreateDefaultValue", &Class::CreateDefaultValue)
        .def("Deserialize", [](const Class* self, py::bytes buffer) {
            std::string str = buffer;
            std::unique_ptr<AbstractValue> value = self->CreateDefaultValue();
            self->Deserialize(str.data(), str.size(), value.get());
            return value;
          }, py::arg("buffer"))
        .def("Serialize", [](
              const Class* self, const AbstractValue* abstract_value) {
            std::vector<uint8_t> bytes;
            self->Serialize(*abstract_value, &bytes);
            return py::bytes(
                reinterpret_cast<const char*>(bytes.data()), bytes.size());
          });
  }

  {
    using Class = LcmPublisherSystem;
    py::class_<Class>(m, "LcmPublisherSystem")
        .def(
            py::init<const std::string&, std::unique_ptr<SerializerInterface>,
                     DrakeLcmInterface*>(),
            py::arg("channel"), py::arg("serializer"), py::arg("lcm"),
            // Keep alive: `self` keeps `DrakeLcmInterface` alive.
            py::keep_alive<1, 3>())
        .def(
            "set_publish_period", &Class::set_publish_period,
            py::arg("period"));
  }

  {
    using Class = LcmSubscriberSystem;
    py::class_<Class>(m, "LcmSubscriberSystem")
        .def(
            py::init<const std::string&, std::unique_ptr<SerializerInterface>,
                     DrakeLcmInterface*>(),
            py::arg("channel"), py::arg("serializer"), py::arg("lcm"),
            // Keep alive: `self` keeps `DrakeLcmInterface` alive.
            py::keep_alive<1, 3>());
  }
}

}  // namespace pydrake
}  // namespace drake
