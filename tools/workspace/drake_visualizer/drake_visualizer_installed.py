"""Runs Drake Visualizer with builtin scripts under an install tree."""

from os.path import isdir, isfile, join, dirname, realpath
import sys

# Manipulate relative path, acommodating the potential use of symlinks.
PREFIX_DIR = dirname(dirname(realpath(__file__)))
assert isdir(join(PREFIX_DIR, "bin")), f"Bad location: {PREFIX_DIR}"
# This is adapted from a sample virtualenv's `activate_this.py` script.
sys.path.insert(0,
    join(PREFIX_DIR, f"lib/python{sys.version[:3]}/site-packages"))

from drake.tools.workspace.drake_visualizer import (
    _exec_drake_visualizer_with_plugins,
)


def main():
    # Execute wrapper.
    _exec_drake_visualizer_with_plugins.main(
        join(SCRIPT_DIR, "drake-visualizer-real"))


assert __name__ == "__main__"
main()
