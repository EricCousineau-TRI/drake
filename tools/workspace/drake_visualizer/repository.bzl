# -*- mode: python -*-
# vi: set ft=python :

"""
Downloads and unpacks a precompiled version of drake-visualizer (a subset of
Director, https://git.io/vNKjq) and makes it available to be used as a
dependency of shell scripts.

Archive naming convention:
    dv-<version>-g<commit>-python-<python version>-qt-<qt version>
        -vtk-<vtk version>-<platform>-<arch>[-<rebuild>]

Build configuration:
    BUILD_SHARED_LIBS=OFF
    CMAKE_BUILD_TYPE=Release
    DD_QT_VERSION=5
    USE_EXTERNAL_INSTALL=ON
    USE_LCM=ON
    USE_LCMGL=ON
    USE_SYSTEM_EIGEN=ON
    USE_SYSTEM_LCM=ON
    USE_SYSTEM_LIBBOT=ON
    USE_SYSTEM_VTK=ON

Example:
    WORKSPACE:
        load(
            "@drake//tools/workspace/drake_visualizer:repository.bzl",
            "drake_visualizer_repository",
        )
        drake_visualizer_repository(name = "foo")

    BUILD:
        sh_binary(
            name = "foobar",
            srcs = ["bar.sh"],
            data = ["@foo//:drake_visualizer"],
        )

Argument:
    name: A unique name for this rule.
"""

load("@drake//tools/workspace:os.bzl", "determine_os")

# TODO(jamiesnape): Publish scripts used to create binaries. There will be a CI
# job for developers to build new binaries on demand.
def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)
    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        archive = "dv-0.1.0-314-ga5a6f6f-python-2.7.15-qt-5.11.1-vtk-8.1.1-mac-x86_64.tar.gz"  # noqa
        sha256 = "02f321cf6068068f1aa9747b6b7834c41cd5ccf53aef90ad58229f2c1bfa963c"  # noqa
    elif os_result.ubuntu_release == "16.04":
        archive = "dv-0.1.0-314-ga5a6f6f-python-2.7.12-qt-5.5.1-vtk-8.1.1-xenial-x86_64.tar.gz"  # noqa
        sha256 = "4bd36e80295006ce4bab57fa57b95b69511623abba80094fb2fdf1eaa18607f9"  # noqa
    elif os_result.ubuntu_release == "18.04":
        archive = "dv-0.1.0-314-ga5a6f6f-python-2.7.15-qt-5.9.5-vtk-8.1.1-bionic-x86_64.tar.gz"  # noqa
        sha256 = "49d4fe29285ebbc420d19bf91511e36e8b1eb03d23bc7912d982ae12c4b2b36c"  # noqa
    else:
        fail("Operating system is NOT supported", attr = os_result)

    urls = [
        x.format(archive = archive)
        for x in repository_ctx.attr.mirrors.get("director")
    ]
    root_path = repository_ctx.path("")

    repository_ctx.download_and_extract(urls, root_path, sha256 = sha256)

    file_content = """# -*- python -*-

# DO NOT EDIT: generated by drake_visualizer_repository()

load("@python//:version.bzl", "PY_SITE_PACKAGES_RELPATH")

licenses([
    "notice",  # Apache-2.0 AND BSD-3-Clause AND Python-2.0
    "reciprocal",  # MPL-2.0
    "restricted",  # LGPL-2.1-only AND LGPL-2.1-or-later AND LGPL-3.0-or-later
    "unencumbered",  # Public-Domain
])

# drake-visualizer has the following non-system dependencies in addition to
# those declared in deps:
#   bot2-lcmgl: LGPL-3.0-or-later
#   ctkPythonConsole: Apache-2.0
#   Eigen: BSD-3-Clause AND MPL-2.0 AND Public-Domain
#   LCM: BSD-3-Clause AND LGPL-2.1-only AND LGPL-2.1-or-later
#   Python: Python-2.0
#   PythonQt: LGPL-2.1-only
#   QtPropertyBrowser: LGPL-2.1-only
# TODO(jamiesnape): Enumerate system dependencies.

py_library(
    name = "drake_visualizer_python_deps",
    deps = [
        "@lcm//:lcm-python",
        "@lcmtypes_bot2_core//:lcmtypes_bot2_core_py",
        # TODO(eric.cousineau): Expose VTK Python libraries here for Linux.
        "@lcmtypes_robotlocomotion//:lcmtypes_robotlocomotion_py",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "drake_visualizer",
    srcs = glob([
        "lib/libPythonQt.*",
        "lib/libddApp.*",
        PY_SITE_PACKAGES_RELPATH + "/bot_lcmgl/**/*.py",
        PY_SITE_PACKAGES_RELPATH + "/director/**/*.py",
        PY_SITE_PACKAGES_RELPATH + "/director/**/*.so",
        PY_SITE_PACKAGES_RELPATH + "/urdf_parser_py/**/*.py",
    ]) + [
        "bin/drake-visualizer",
        "share/doc/director/LICENSE.txt",
    ],
    data = [
        ":drake_visualizer_python_deps",
        "@lcm//:libdrake_lcm.so",
        "@vtk",
    ],
    visibility = ["//visibility:public"],
)

load("@drake//tools/install:install.bzl", "install_files")
install_files(
    name = "install",
    dest = ".",
    files = [":drake_visualizer"],
    visibility = ["//visibility:public"],
)
"""

    repository_ctx.file(
        "BUILD.bazel",
        content = file_content,
        executable = False,
    )

drake_visualizer_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
