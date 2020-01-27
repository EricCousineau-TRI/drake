# -*- python -*-

load("//tools/workspace:generate_file.bzl", "generate_file")
load("@drake//tools/workspace:github.bzl", "github_archive")

# HACK
_REPOSITORY = "pybind/pybind11"

_COMMIT = "07e225932235ccb0db5271b0874d00f086f28423"

_SHA256 = "503d8cc92591fab601eb7adc2c9a725cebfdededcf81036a486d932d14376100"

def pybind11_repository(
        name,
        mirrors = None):
    native.new_local_repository(
        name = name,
        path = "/home/eacousineau/proj/tri/repo/externals/pybind11",
        build_file = "@drake//tools/workspace/pybind11:package.BUILD.bazel",
    )
    # github_archive(
    #     name = name,
    #     repository = _REPOSITORY,
    #     commit = _COMMIT,
    #     sha256 = _SHA256,
    #     build_file = "@drake//tools/workspace/pybind11:package.BUILD.bazel",
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
