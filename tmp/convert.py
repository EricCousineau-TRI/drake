#!/usr/bin/env python3

import os
from os.path import basename, dirname, abspath
import re
from subprocess import run, PIPE, STDOUT


def ign_sdf(cmd, sdf_file, **kwargs):
    ign_sdf = abspath("bazel-bin/tools/workspace/sdformat/ign_sdf")
    return run(
        [ign_sdf, f"--{cmd}", sdf_file],
        stdout=PIPE,
        encoding="utf8",
        **kwargs
    )


def check(file):
    result = ign_sdf(
        "check",
        basename(file),
        cwd=dirname(file),
        stderr=STDOUT,
    )
    print(result.returncode)
    print(result.stdout)


def convert(file):
    result = ign_sdf(
        "print",
        file,
        stderr=PIPE,
    )
    if result.returncode == 0:
        with open(file, "w") as f:
            f.write(result.stdout)
    else:
        print(result.returncode)
        print(result.stderr)


class Replace:
    def __init__(self, from_, to_):
        self.from_ = re.compile(from_)
        self.to_ = to_

    def __call__(self, text):
        return re.sub(self.from_, self.to_, text)


reps = [
    # Replace(
    #     from_=r""" <sdf (.*?)version=('.*?'|".*?") """.strip(),
    #     to_=r""" <sdf \1version="1.7" """.strip(),
    # ),
    Replace(
        from_=r"pose frame=(''|\"\")",
        to_="pose",
    ),
    Replace(
        from_=r"<\?xml.*?\?>\n",
        to_="",
    ),
    Replace(
        from_=r""" "(.*?)" """.strip(),
        to_=r""" '\1' """.strip(),
    ),
    Replace(
        from_=" -0 ",
        to_=" 0 ",
    ),
    Replace(
        from_=r"<pose(.*?)>\s*0\s+0\s+0\s+0\s+0\s+0</pose>",
        to_=r"<pose\1/>",
    ),
    Replace(
        from_=r" *<pose/>\n",
        to_="",
    ),
    # Replace(
    #     from_=r" +\n",
    #     to_="\n",
    # )
]


def perl_pie(file):
    with open(file, encoding="utf8") as f:
        text = f.read().strip()
    for rep in reps:
        text = rep(text)
    with open(file, "w", encoding="utf8") as f:
        f.write('<?xml version=\'1.0\'?>\n')
        f.write(text.strip() + "\n")


def gut_check(file):
    with open(file, encoding="utf8") as f:
        text = f.read().strip()
    version, = re.findall(r""" <sdf .*?version=['"](.*?)['"] """.strip(), text)
    if version == "1.7":
        assert "use_parent_model_frame" not in text


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
        print(file)
        gut_check(file)
        # check(file)
        # convert(file)
        # perl_pie(file)
        print("---")


assert __name__ == "__main__"
main()
