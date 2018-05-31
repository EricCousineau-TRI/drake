# -*- mode: python -*-
# vi: set ft=python :

"""
Provides NumPy from a wheel file.

Example:
    WORKSPACE:
        load("@drake//tools/workspace/numpy:repo.bzl", "numpy_py_repository")
        numpy_py_repository(
            name = "foo",
        )

    BUILD:
        py_library(
            name = "foobar",
            deps = ["@foo//:numpy_py"],
            srcs = ["bar.py"],
        )

Arguments:
    name: A unique name for this rule.
"""

load("@drake//tools/workspace:os.bzl", "determine_os")

# Generated from the presently unmerged numpy/numpy#10898.
# See `./packaging/README.md` for instructions to generate these binaries.
# N.B. When you update the version of NumPy, ensure to update the variable
# `expected_version` in `numpy_install_test.py` to reflect the *exact* version
# per NumPy's build (which should be part of the *.whl's filename).
# PR DRAFT(eric.cousineau): Upload these to S3 when they pass review.

def _get_url(platform):
    git_ref = "6721890"
    base_format = "https://github.com/EricCousineau-TRI/experimental/raw/6109720315a1efae945fc6fa1227200b9f7782a2/numpy/numpy-1.15.0.dev0%2B{git_ref}-cp27-{platform}.whl"  # noqa
    return base_format.format(git_ref=git_ref, platform=platform)

wheels = {
    "ubuntu_16.04": {
        "url": _get_url("cp27mu-linux_x86_64"),
        "sha256": "d4faa33356481f7e3a2930a2fb7f0dcead75f976b61e5ab88624cb40aca46c68",  # noqa
    },
    "mac": {
        "url": _get_url("cp27m-macosx_10_13_x86_64"),
        "sha256": "32d6a833b0543373c9e3ee291417a6a5faedd33f1669e38845ab92a89431db8e",  # noqa
    },
}

def _impl(repository_ctx):
    # Do not name this `numpy`, as Bazel will confuse `PYTHONPATH`:
    # If `numpy` is located in the repo root, `random` will be shadowed by
    # `numpy.random`, causing everything to fail.
    # If installed elsewhere, Bazel will put an autogenerated `__init__.py` at
    # external directory's root, leak the wrong path, and shadow the real
    # `numpy`. See https://github.com/bazelbuild/bazel/issues/3998
    if repository_ctx.name == "numpy":
        fail("Do not name this repository `numpy`. Please name it " +
             "`numpy_py` or something else.")

    os_result = determine_os(repository_ctx)
    if os_result.error != None:
        fail(os_result.error)

    wheel = None
    if os_result.is_macos:
        wheel = wheels["mac"]
    elif os_result.is_ubuntu:
        key = "ubuntu_" + os_result.ubuntu_release
        wheel = wheels.get(key)
    if wheel == None:
        fail("Unsupported platform")

    repository_ctx.download_and_extract(
        url = wheel["url"],
        sha256 = wheel["sha256"],
        type = "zip",
    )
    repository_ctx.symlink(
        Label("@drake//tools/workspace/numpy:package.BUILD.bazel"),
        "BUILD.bazel")

numpy_py_repository = repository_rule(
    _impl,
)
