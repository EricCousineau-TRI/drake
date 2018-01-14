load(
    "//tools/external_data:expose_all_files.bzl",
    "recursive_filegroup",
)
load(
    "//tools/external_data/test:workspace_test.bzl",
    "workspace_test",
)

_workspace_list = [
    "external_data_bazel_pkg_test",
]

_upstream_files = [
    "//tools/external_data:all_files_recursive",
]

def external_data_workspace_test(
        name,
        args = ["'bazel test //...'"],
        data = []):
    package = "@{}//".format(name)
    package_files = package + ":all_files_recursive"
    pkg_reldir = "external/" + name
    script = "external_data_workspace_test.sh"
    workspace_test(
        name = name,
        cmd = "$(location {})".format(script),
        args = [
            pkg_reldir,
        ] + args,
        data = [
            package_files,
            script,
        ] + _upstream_files + data,
    )

def external_data_lint_files():
    """
    Creates a recursive_filegroup "all_${type}_files" (where `type` can be
    "bazel_lint" or "python_lint"), such that they can be consumed by
    `add_lint_tests`.
    """
    packages = ["//tools/external_data"]
    for workspace in _workspace_list:
        package = "@" + workspace + "//"
        # Prepare to expose all files recursively.
        packages.append(package)
    # Join files for linting, to permit $(locations ...) expansion directly
    # on transitive file dependencies.
    for name in ["bazel_lint_files", "python_lint_files"]:
        data = [package + ":" + name + "_recursive" for package in packages]
        recursive_filegroup(
            name = "all_" + name,
            data = data,
        )

def external_data_test_repositories(workspace_dir):
    """
    Adds test workspace directories as repositories so that their files can be
    consumed and their tests can be ignored by `bazel test ...` from Drake.
    """
    # WARNING: Bazel also craps out here if `workspace_dir + path` is used
    # rather than just `path`.
    # N.B. This error is *stateful*. You will get different behavior depending
    # on what has been built / run previously in Bazel. In one mode, the error
    # will be:
    #   Encountered error while [...]
    #   /home/${USER}/.cache/bazel/_bazel_${USER}/${HASH}/external/external_data_bazel_pkg_test  # noqa
    #   must  be an existing directory
    # In another mode, you will get Java errors:
    #   java.lang.IllegalArgumentException: PathFragment
    #   tools/external_data/workspace is not beneath
    #   /home/${USER}/${WORKSPACE_DIR}/tools/external_data/workspace
    for workspace in _workspace_list:
        native.local_repository(
            name = workspace,
            path = "tools/external_data/test/" + workspace,
        )
