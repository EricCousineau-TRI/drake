# -*- python -*-

load("//tools/workspace:generate_file.bzl", "generate_file")
load("@drake//tools/workspace:github.bzl", "github_archive")

# Using the `drake` branch of this repository.
_REPOSITORY = "RobotLocomotion/pybind11"

_COMMIT = "9353788ef70f106dc687e09987d503b7e463db0f"

# DNM PR DRAFT: https://github.com/RobotLocomotion/pybind11/pull/39
_SHA256 = "57f4883a22ebb9dca1a3b41ad550de05d80cadd503ef1c9f4ba5118a6d6bf3f6"

def pybind11_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = _REPOSITORY,
        commit = _COMMIT,
        sha256 = _SHA256,
        build_file = "@drake//tools/workspace/pybind11:package.BUILD.bazel",
        mirrors = mirrors,
    )

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
