# -*- mode: python -*-
# vi: set ft=python :

"""
Finds local system Python headers and libraries using python-config and
makes them available to be used as a C/C++ dependency. On macOS, Python
libraries should not typically be directly linked, so the :python target passes
the "-undefined dynamic_lookup" linker flag, however in the rare cases that
this would cause an undefined symbol error, a :python_direct_link target is
provided. On Linux, these targets are identical.

Example:
    WORKSPACE:
        load("@drake//tools/workspace/python:repository.bzl", "python_repository")  # noqa
        python_repository(
            name = "foo",
            version = "2",
        )

    BUILD:
        cc_library(
            name = "foobar",
            deps = ["@foo//:python"],
            srcs = ["bar.cc"],
        )

Arguments:
    name: A unique name for this rule.
    version: The major or major.minor version of Python headers and libraries
    to be found.
"""

load("@drake//tools/workspace:execute.bzl", "which")
load("@drake//tools/workspace:os.bzl", "determine_os")

def _exec_ctx(repository_ctx, bin):
    return struct(ctx=repository_ctx, bin=bin)

def _exec(exec_ctx, args, strip=True):
    args = [exec_ctx.bin] + args
    result = exec_ctx.ctx.execute(args)
    if result.return_code != 0:
        fail("Could not execute {}: {}".format(args, result.stderr))
    return result.stdout.strip()

_SUPPORT_MATRIX = {
    "ubuntu:16.04": ["2.7", "3.5"],
    "ubuntu:18.04": ["2.7", "3.5"],
    "macOS:10.13": ["2.7", "3.6"],
    "macOS:10.14": ["2.7", "3.6"],
}

def _impl(repository_ctx):
    version = repository_ctx.attr.version
    if version == "bazel":
        tmp_ctx = _exec_ctx(repository_ctx, repository_ctx.which("python"))
        version = _exec(
            tmp_ctx, ["-c", "import sys; print(sys.version_info.major)"])

    python_config = repository_ctx.which("python{}-config".format(version))
    if not python_config:
        fail("Could NOT find python{}-config".format(
            version,
        ))
    python = repository_ctx.which("python{}".format(version))

    py_ctx = _exec_ctx(repository_ctx, python)
    config_ctx = _exec_ctx(repository_ctx, python_config)

    # Estimate that we're using the same configuration between
    # `python{version}` and `python-config{version}`.
    py_configdir = _exec(
        py_ctx,
        ["-c", "import sysconfig; print(sysconfig.get_config_var(\"LIBPL\"))"])
    py_config_configdir = _exec(config_ctx, ["--configdir"])
    if (py_configdir != py_config_configdir):
        fail("Mismatch between {} and {}: {} != {}".format(
            python,
            python_config,
            py_configdir,
            py_config_configdir,
        ))

    os_result = determine_os(repository_ctx)
    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        os_key = os_result.distribution + ":" + os_result.macos_release
    else:
        os_key = os_result.distribution + ":" + os_result.ubuntu_release
    supported = _SUPPORT_MATRIX[os_key]

    version_major_minor = _exec(
        py_ctx,
        ["-c", "from sys import version_info as v; print(\"{}.{}\"" +
               ".format(v.major, v.minor))"])
    if version_major_minor not in supported:
        msg = (
            "Python {} is not a supported / tested version for use with " +
            "Drake.\n  Supported versions: {}\n"
        ).format(version_major_minor, supported)
        fail(msg)

    cflags = _exec(config_ctx, ["--includes"]).split(" ")
    cflags = [cflag for cflag in cflags if cflag]

    root = repository_ctx.path("")
    root_len = len(str(root)) + 1
    base = root.get_child("include")

    includes = []

    for cflag in cflags:
        if cflag.startswith("-I"):
            source = repository_ctx.path(cflag[2:])
            destination = base.get_child(str(source).replace("/", "_"))
            include = str(destination)[root_len:]

            if include not in includes:
                repository_ctx.symlink(source, destination)
                includes += [include]

    linkopts = _exec(config_ctx, ["--ldflags"]).split(" ")
    linkopts = [linkopt for linkopt in linkopts if linkopt]

    for i in reversed(range(len(linkopts))):
        if not linkopts[i].startswith("-"):
            linkopts[i - 1] += " " + linkopts.pop(i)

    linkopts_direct_link = list(linkopts)

    if os_result.is_macos:
        for i in reversed(range(len(linkopts))):
            if linkopts[i].find("python{}".format(version)) != -1:
                linkopts.pop(i)
        linkopts = ["-undefined dynamic_lookup"] + linkopts

    sys_prefix = _exec(py_ctx, ["-c", "import sys; print(sys.prefix)"])

    file_content = """# -*- python -*-

# DO NOT EDIT: generated by python_repository()

licenses(["notice"])  # Python-2.0 / Python-3.0

# Only include first level of headers included from `python_repository`
# (`include/<destination>/*`). This should exclude third party C headers which
# may be nested within `/usr/include/python2.7`, such as `numpy` when installed
# via `apt` on Ubuntu.
headers = glob(
    ["include/*/*"],
    exclude_directories = 1,
)

cc_library(
    name = "python_headers",
    hdrs = headers,
    includes = {},
    visibility = ["//visibility:private"],
)

cc_library(
    name = "python",
    linkopts = {},
    deps = [":python_headers"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "python_direct_link",
    linkopts = {},
    deps = [":python_headers"],
    visibility = ["//visibility:public"],
)
    """.format(includes, linkopts, linkopts_direct_link)

    repository_ctx.file(
        "BUILD.bazel",
        content = file_content,
        executable = False,
    )

    skylark_content = """

# DO NOT EDIT: generated by python_repository()
def python_version_major_minor():
    return "{version}"

def python_lib_dir():
    return "lib/python{version}"

def python_sys_prefix():
    return "{prefix}"
""".format(version=version_major_minor, prefix=sys_prefix)
    repository_ctx.file(
        "python.bzl", content = skylark_content, executable = False)

    print(sys_prefix)

python_repository = repository_rule(
    _impl,
    attrs = {
        "version": attr.string(default = "bazel"),
    },
    local = True,
)
