#!/usr/bin/env python3

import os
from os.path import basename, dirname, abspath
import re
from subprocess import run, PIPE, STDOUT


def xmllint(file):
    run(["xmllint", file, "--format", "-o", file], check=True)


class Replace:
    def __init__(self, from_, to_):
        self.from_ = re.compile(from_, flags=re.MULTILINE | re.DOTALL)
        self.to_ = to_

    def __call__(self, text):
        return re.sub(self.from_, self.to_, text)


def simplify_empty_pose(m):
    str_tuple = m.group(2).strip().split()
    try:
        float_tuple = [float(s) for s in str_tuple]
    except ValueError:
        float_tuple = None
    if float_tuple is not None and all(x == 0. for x in float_tuple):
        return f"<pose{m.group(1)}/>"
    else:
        return m.group(0)


reps = [
    Replace(
        from_=r"pose frame=(''|\"\")",
        to_="pose",
    ),
    Replace(
        from_=" -0 ",
        to_=" 0 ",
    ),
    Replace(
        from_=r"<pose(.*?)>(.*?)</pose>",
        to_=simplify_empty_pose,
    ),
    Replace(
        from_=r" *<pose/>\n",
        to_="",
    ),
]


def perl_pie(file):
    with open(file, encoding="utf8") as f:
        text = f.read().strip()
    for rep in reps:
        text = rep(text)
    with open(file, "w", encoding="utf8") as f:
        f.write(text.rstrip() + "\n")


find_cmd = r"""
find . \
    -name '*.config' \
    -o -name '*.sdf' \
    -o -name '*.urdf' \
    -o -name '*.xacro' \
    -o -name '*.xml' \
    -o -name '*.world'
"""

# First, check all models.
def main():
    os.chdir(dirname(abspath(__file__)) + "/..")

    files = run(
        find_cmd,
        stdout=PIPE,
        encoding="utf8",
        check=True,
        shell=True,
    ).stdout.strip().splitlines()
    files.sort()

    for file in list(files):
        if file.startswith("./attic/"):
            files.remove(file)

    for file in files:
        print(file)
        perl_pie(file)
        if not file.endswith("xacro"):
            xmllint(file)
        print("---")


assert __name__ == "__main__"
main()
