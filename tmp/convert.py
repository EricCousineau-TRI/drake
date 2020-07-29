#!/usr/bin/env python3

import os
from os.path import basename, dirname, abspath
from subprocess import run, PIPE, STDOUT

def ign_sdf(cmd, sdf_file, **kwargs):
    ign_sdf = abspath("bazel-bin/tools/workspace/sdformat/ign_sdf")
    return run(
        [ign_sdf, f"--{cmd}", sdf_file],
        stdout=PIPE,
        encoding="utf8",
        **kwargs
    )

# First, check all models.
def main():
    os.chdir(dirname(abspath(__file__)) + "/..")

    files = run(
        ["find", ".", "-name", "*.sdf"],
        stdout=PIPE,
        encoding="utf8",
        check=True,
    ).stdout.strip().splitlines()
    files.sort()
    for file in list(files):
        if file.startswith("./attic/"):
            files.remove(file)
    
    for file in files:
        result = ign_sdf(
            "check",
            basename(file),
            cwd=dirname(file),
            stderr=STDOUT,
        )
        print(file)
        print(result.returncode)
        print(result.stdout)
        print("---")


assert __name__ == "__main__"
main()
