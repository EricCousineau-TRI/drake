#include <gflags/gflags.h>

#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/geometry/render/camera_properties.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"

DEFINE_bool(use_primer, true, "");
DEFINE_int32(count, 2, "");
DEFINE_int32(render_count, 3, "");

using namespace drake;
using namespace drake::geometry;
using namespace drake::geometry::render;
using namespace drake::systems::sensors;

void EmptyRender(int render_count) {
  auto renderer = MakeRenderEngineVtk(RenderEngineVtkParams());
  CameraProperties camera_prop(
      640, 480, M_PI / 4, "doesn't matter");
  ImageRgba8U image(camera_prop.width, camera_prop.height);

  for (int r = 0; r < render_count; ++r) {
    drake::log()->info("  Render {}", r);
    renderer->RenderColorImage(camera_prop, false, &image);
  }
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::unique_ptr<RenderEngine> renderer;
  if (FLAGS_use_primer) {
    drake::log()->info("Priming...");
    renderer = MakeRenderEngineVtk(RenderEngineVtkParams());
    CameraProperties camera_prop(4, 3, M_PI / 4, "");
    ImageRgba8U image(camera_prop.width, camera_prop.height);
    renderer->RenderColorImage(camera_prop, false, &image);
  }

  for (int i = 0; i < FLAGS_count; ++i) {
    drake::log()->info("i: {}", i);
    EmptyRender(FLAGS_render_count);
  }
  drake::log()->info("[ Done ]");
  return 0;
}
