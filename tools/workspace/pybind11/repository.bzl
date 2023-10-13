load("//tools/workspace:generate_file.bzl", "generate_file")
load("//tools/workspace:github.bzl", "github_archive")

# Using the `drake` branch of this repository.
_REPOSITORY = "RobotLocomotion/pybind11"

# When upgrading this commit, check the version header within
#  https://github.com/RobotLocomotion/pybind11/blob/drake/include/pybind11/detail/common.h
# and if it has changed, then update the version number in the two
# pybind11-*.cmake files in the current directory to match.

# DNM PR: https://github.com/RobotLocomotion/pybind11/pull/63

_COMMIT = "e61bb561a67037ca2ddcb1386f44a6014c8091e4"

_SHA256 = "dbcddb96dd5db5f24d00c8fc04d5d131166150c5c9730d37b721f8298dacf4f1"

def pybind11_repository(
        name,
        mirrors = None):
    native.new_local_repository(
        name = name,
        path = "/home/eacousineau/proj/tri/repo/externals/pybind11",
        build_file = "//tools/workspace/pybind11:package.BUILD.bazel",
    )

    # github_archive(
    #     name = name,
    #     repository = _REPOSITORY,
    #     commit = _COMMIT,
    #     sha256 = _SHA256,
    #     build_file = ":package.BUILD.bazel",
    #     mirrors = mirrors,
    # )

def generate_pybind11_version_py_file(name):
    vars = dict(
        repository = repr(_REPOSITORY),
        commit = repr(_COMMIT),
        sha256 = repr(_SHA256),
    )
    generate_file(
        name = name,
        content = '''
"""
Provides information on the external fork of `pybind11` used by `pydrake`.
"""

repository = {repository}
commit = {commit}
sha256 = {sha256}
'''.format(**vars),
    )
