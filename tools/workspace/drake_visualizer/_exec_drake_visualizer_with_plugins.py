"""Runs Drake Visualizer with builtin scripts."""
import argparse
import os
import sys

from drake.tools.workspace.drake_visualizer.plugin import (
    __file__ as plugin_init_file,
)


def main(drake_visualizer_real):
    assert os.path.isfile(drake_visualizer_real), (
        f"Must exist: {drake_visualizer_real}")

    parser = argparse.ArgumentParser(
        prog="Drake Modifications",
        description=__doc__,
        add_help=False,
    )
    parser.add_argument(
        "--use_builtin_scripts", type=str, default="all",
        help="Use Drake's builtin scripts in conjunction with Drake "
             "Visualizer.")
    args, argv = parser.parse_known_args(sys.argv[1:])
    # drake-visualizer greedily consumes arguments, so we must catch them
    # first and pass them as an environment variable.
    os.environ["_DRAKE_VISUALIZER_BUILTIN_SCRIPTS"] = args.use_builtin_scripts
    use_builtin_scripts_file = os.path.join(
        os.path.dirname(plugin_init_file), "use_builtin_scripts.py")

    if "-h" in argv or "--help" in argv:
        parser.print_help()
        print()

    exec_args = (
        [drake_visualizer_real] + argv +
        ["--script", use_builtin_scripts_file])
    os.execv(drake_visualizer_real, exec_args)
    assert False, "Should not reach here"
