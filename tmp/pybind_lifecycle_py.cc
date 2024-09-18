/**
Manual lifecycle management. For more information, see
https://github.com/RobotLocomotion/drake/issues/14387
and accompanying `pybind_lifecycle_test.py`.
*/

#include <vector>

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace drake {
namespace {

using PatientList = std::vector<PyObject*>;

/**
Returns pointer to patients for an object, if they exist.
Returns null otherwise.
*/
PatientList* GetPatients(PyObject* obj) {
  auto* instance = reinterpret_cast<py::detail::instance *>(obj);
  auto& patients_map = py::detail::get_internals().patients;
  auto pos = patients_map.find(obj);
  if (pos == patients_map.end()) {
    return nullptr;
  }
  if (!instance->has_patients) {
    return nullptr;
  }
  return &pos->second;
}

/** pybind wrapper of the above. */
py::list GetPatientsPy(py::object obj) {
  py::list out;
  PatientList* patients = GetPatients(obj.ptr());
  if (patients) {
    for (PyObject* patient : *patients) {
      out.append(py::handle(patient));
    }
  }
  return out;
}

void ClearPatientsImpl(PatientList* patients) {
  for (PyObject *patient : *patients) {
    Py_CLEAR(patient);
  }
  patients->clear();
}

/**
Like pybind11::detail::clear_patients(); however:
- This simply "resets" patients.
- This will skip any Python object that is either not a pybind-registered
  object or does not have any lifesupport enabled (whereas the original will
  fail fast).

WARNING: This is for *expert* users only. If you are unsure what this does, do
not change calls to this function. You may cause segfaults. If you do, you may
be on your own.
*/
void ClearPatients(py::handle obj) {
  auto* patients = GetPatients(obj.ptr());
  if (!patients) {
    return;
  }
  ClearPatientsImpl(patients);
}

PYBIND11_MODULE(pybind_lifecycle, m) {
  m.def("GetPatients", &GetPatientsPy, py::arg("obj"));
  m.def("ClearPatients", &ClearPatients, py::arg("obj"));
}

}  // namespace
}  // namespace drake
