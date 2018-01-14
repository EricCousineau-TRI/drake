# -*- python -*-

# Needs quotes, because `sh_test(args = [...])` just concatenates the arguments.
_CMD_DEFAULT = "'bazel test //...'"

def workspace_test(
        name,
        cmd = _CMD_DEFAULT,
        data = []):
    """Provides a unittest access to a given workspace
    contained in the current project.

    @param cmd
        Command to run. By default is `bazel test //...`.
    @param data
        Additional data (e.g. other workspaces).
    """
    anchor = name + "_anchor"
    all_files = name + "_all_files"
    args = [cmd, "$(location {})".format(anchor)]
    native.sh_test(
        name = name,
        # TODO(eric.cousineau): Is it possible get the package of the *current*
        # macro file (rather than the current BUILD file)?
        srcs = [":workspace_writeable_test.sh"],
        args = args,
        data = [anchor, all_files] + data,
    )
