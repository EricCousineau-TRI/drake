/** @file
Provides a C++ version libsdformat's `ign sdf` ruby command-line interface.
 */

#include <stdexcept>
#include <string>

#include <gflags/gflags.h>

DEFINE_string(
    check, "",
    "Checks an SDFormat file");
DEFINE_string(
    describe, "",
    "Describes a given SDFormat version");
DEFINE_string(
    print, "",
    "Prints an SDFormat file after parsing and converting to the converted "
    "to the latest specification");

extern "C" int cmdCheck(const char *_path);
extern "C" int cmdDescribe(const char *_version);
extern "C" int cmdPrint(const char *_path);

namespace drake {
namespace {

int DoMain() {
  // All flags are mutually exclusive.
  const int num_top_level_options =
      !FLAGS_check.empty() + !FLAGS_print.empty() + !FLAGS_describe.empty();
  if (num_top_level_options != 1) {
    throw std::runtime_error("Must provide only one of the given options");
  }

  if (!FLAGS_check.empty()) {
    return cmdCheck(FLAGS_check.c_str());
  } else if (!FLAGS_describe.empty()) {
    return cmdDescribe(FLAGS_describe.c_str());
  } else if (!FLAGS_print.empty()) {
    return cmdPrint(FLAGS_print.c_str());
  }
  return 1;
}

}  // namespace
}  // namespace drake

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::DoMain();
}
