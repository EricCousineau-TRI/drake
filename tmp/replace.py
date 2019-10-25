#!/usr/bin/env python3
import os
from os.path import basename, dirname, abspath, join, isfile, splitext
from subprocess import run, check_output
import multiprocessing as mp
import re


files = [
    # "attic/manipulation/scene_generation/test/test_random_clutter_to_rest.cc",
    # "attic/multibody/collision/test/collision_filter_group_test.cc",
    # "attic/multibody/collision/test/model_test.cc",
    # "attic/multibody/parsers/test/urdf_parser_test.cc",
    # "attic/multibody/parsers/test/xml_util_test.cc",
    # "attic/multibody/rigid_body_plant/test/compliant_contact_model_test.cc",
    # "attic/multibody/rigid_body_plant/test/drake_visualizer_test.cc",
    # "attic/multibody/rigid_body_plant/test/rigid_body_plant_test.cc",
    # "attic/multibody/test/rigid_body_constraint_test.cc",
    # "attic/multibody/test/rigid_body_tree/rigid_body_tree_kinematics_test.cc",
    # "attic/systems/sensors/test/rgbd_camera_test.cc",
    # "attic/systems/sensors/test/rgbd_renderer_ospray_test.cc",
    "common/test/autodiffxd_make_coherent_test.cc",
    "common/test/drake_throw_test.cc",
    "common/test/eigen_types_test.cc",
    "common/test/find_resource_test.cc",
    "common/test/polynomial_test.cc",
    "common/test/symbolic_formula_test.cc",
    "common/test/symbolic_ldlt_test.cc",
    "common/test/symbolic_polynomial_test.cc",
    "common/test/type_safe_index_test.cc",
    "common/test/value_test.cc",
    "common/trajectories/test/piecewise_polynomial_generation_test.cc",
    "examples/compass_gait/test/compass_gait_test.cc",
    "examples/manipulation_station/test/manipulation_station_test.cc",
    "examples/rod2d/test/rod2d_test.cc",
    "geometry/proximity/test/distance_sphere_to_shape_test.cc",
    "geometry/proximity/test/hydroelastic_callback_test.cc",
    "geometry/render/test/render_engine_vtk_test.cc",
    "geometry/render/test/render_label_test.cc",
    "geometry/test/frame_kinematics_vector_test.cc",
    "geometry/test/geometry_properties_test.cc",
    "geometry/test/geometry_set_test.cc",
    "geometry/test/geometry_state_test.cc",
    "geometry/test/identifier_test.cc",
    "geometry/test/internal_geometry_test.cc",
    "geometry/test/scene_graph_test.cc",
    "geometry/test/shape_specification_test.cc",
    "manipulation/perception/test/optitrack_pose_extractor_test.cc",
    "manipulation/util/test/moving_average_filter_test.cc",
    "math/test/autodiff_test.cc",
    "math/test/discrete_lyapunov_equation_test.cc",
    "math/test/rigid_transform_test.cc",
    "math/test/rotation_matrix_test.cc",
    "multibody/parsing/test/detail_urdf_geometry_test.cc",
    "multibody/plant/test/floating_body_test.cc",
    "multibody/plant/test/multibody_plant_tamsi_test.cc",
    "multibody/plant/test/multibody_plant_test.cc",
    "multibody/plant/test/tamsi_solver_test.cc",
    "multibody/tree/test/articulated_body_inertia_test.cc",
    "multibody/tree/test/multibody_tree_creation_test.cc",
    "multibody/tree/test/multibody_tree_test.cc",
    "multibody/tree/test/rotational_inertia_test.cc",
    "multibody/tree/test/tree_from_mobilizers_test.cc",
    "multibody/tree/test/weld_joint_test.cc",
    "perception/test/point_cloud_flags_test.cc",
    "perception/test/point_cloud_test.cc",
    "solvers/test/gurobi_solver_grb_license_file_test.cc",
    "solvers/test/linear_complementary_problem_test.cc",
    "solvers/test/mathematical_program_test.cc",
    "solvers/test/mosek_solver_moseklm_license_file_test.cc",
    "solvers/test/nonlinear_program_test.cc",
    "solvers/test/solver_options_test.cc",
    "solvers/test/sos_constraint_test.cc",
    "systems/analysis/test/explicit_euler_integrator_test.cc",
    "systems/analysis/test/hermitian_dense_output_test.cc",
    "systems/analysis/test_utilities/explicit_error_controlled_integrator_test.h",
    "systems/analysis/test_utilities/implicit_integrator_test.h",
    "systems/controllers/test/linear_quadratic_regulator_test.cc",
    "systems/controllers/test/pid_controlled_system_test.cc",
    "systems/controllers/test/pid_controller_test.cc",
    "systems/framework/test/cache_entry_test.cc",
    "systems/framework/test/cache_test.cc",
    "systems/framework/test/dependency_tracker_test.cc",
    "systems/framework/test/diagram_builder_test.cc",
    "systems/framework/test/diagram_context_test.cc",
    "systems/framework/test/diagram_test.cc",
    "systems/framework/test/leaf_context_test.cc",
    "systems/framework/test/leaf_system_test.cc",
    "systems/framework/test/output_port_test.cc",
    "systems/framework/test/system_base_test.cc",
    "systems/framework/test/system_test.cc",
    "systems/framework/test/value_checker_test.cc",
    "systems/framework/test/vector_system_test.cc",
    "systems/primitives/test/gain_test.cc",
    "systems/primitives/test/linear_system_test.cc",
    "systems/primitives/test/signal_logger_test.cc",
]

assert __name__ == "__main__"
os.chdir(join(dirname(abspath(__file__)), ".."))
clang_format = "clang-format-6.0"
reformat_bin = "bazel-bin/tools/lint/clang-format-includes"
add_include = '#include "drake/common/test_utilities/expect_no_throw.h"\n'

def clang_format_changed_lines_only(file, context=3):
    diff = check_output(["git", "diff", file]).decode("utf8")
    with open(file) as f:
        num_lines = len(f.readlines())
    ms = re.findall(r"@@ \-\d+,\d+ \+(\d+),", diff)
    args = []
    for m in ms:
        line = int(m)
        start = max(1, line - context)
        end = min(num_lines, line + context)
        args += [f"-lines={start}:{end}"]
    run([clang_format, "-i", "-style=file"] + args + [file], check=True)


def print_build_file(file):
    base, _ = splitext(basename(file))
    d = dirname(file)
    possible = [
        join(d, "BUILD.bazel"),  # Nominal
        join(dirname(d), "BUILD.bazel"),  # Test
    ]
    for build in possible:
        if not isfile(build):
            continue
        with open(build) as f:
            for i, line in enumerate(f.readlines()):
                if f'"{base}"' in line:
                    print(f"{build}:{i + 1}: {base}")


def func(file):
    print(file)
    with open(file) as f:
        text = f.read()
    if add_include in text:
        print(" - skip")
        return
    text = text.replace("EXPECT_NO_THROW", "DRAKE_EXPECT_NO_THROW")
    text = text.replace("ASSERT_NO_THROW", "DRAKE_ASSERT_NO_THROW")
    text = add_include + text
    with open(file, "w") as f:
        f.write(text)
    clang_format_changed_lines_only(file)
    run([reformat_bin, file], check=True)
    # Print corresponding buildfile + line
    print_build_file(file)

# func("geometry/test/geometry_state_test.cc")
with mp.Pool(processes=20) as pool:
    pool.map(func, files)

print("Done")
