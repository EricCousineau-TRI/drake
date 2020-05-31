#!/usr/bin/env python3

from enum import Enum
from os.path import abspath, dirname, join, basename
from subprocess import run, STDOUT, PIPE


class Type:
    OTHER = ""
    TRIPLE_SLASH = "///"
    DOUBLE_STAR = "/**"
    SINGLE_STAR = "* "
    COMMENT_END = "*/"

    PRIMARY_TYPES = (TRIPLE_SLASH, DOUBLE_STAR)
    SECONDARY_TYPES = (OTHER, SINGLE_STAR)

    @staticmethod
    def parse(text):
        start_type = Type.OTHER
        end_type = Type.OTHER
        if len(text) > 0:
            # Text must already be stripped..
            assert text[0] != " ", repr(text)
            assert text[-1] != " "
            if text.startswith("///"):
                start_type = Type.TRIPLE_SLASH
            elif text.startswith("/**"):
                start_type = Type.DOUBLE_STAR
            elif text.startswith("* "):
                start_type = Type.SINGLE_STAR
            if text.endswith("*/"):
                end_type = Type.COMMENT_END
        return start_type, end_type


def split_indent(line):
    i = 0
    for i in range(len(line)):
        c = line[i]
        assert c != '\t'
        if c != ' ':
            break
    return line[:i], line[i:]


class Docline:
    def __init__(self, filename, num, raw_line):
        self.filename = filename
        self.num = num
        assert raw_line[-1] == "\n"
        self.raw_line = raw_line[:-1]
        self.indent, without_indent = split_indent(self.raw_line)
        self.start_type, self.end_type = Type.parse(without_indent)
        # print(f"{(self.start_type, self.end_type)}: {raw_line}")
        self.text = None
        if self.start_type != Type.OTHER:
            self.text = without_indent[len(self.start_type):]
            if self.end_type == Type.COMMENT_END:
                self.text = self.text[:-len("*/")].rstrip("*")

    def __str__(self):
        return f"{self.filename}:{self.num:<5}: {self.raw_line}"


class Docstring:
    def __init__(self, line):
        assert isinstance(line, Docline)
        self.lines = [line]
        self.primary_type = line.start_type
        self.secondary_type = Type.OTHER
        self._done = False

    def maybe_add_next_line(self, line):
        add_line = False
        if self.primary_type == Type.TRIPLE_SLASH:
            if line.start_type == Type.TRIPLE_SLASH:
                add_line = True
            else:
                self._done = True
        elif self.primary_type == Type.DOUBLE_STAR:
            if line.start_type in Type.SECONDARY_TYPES:
                if len(self.lines) == 1:
                    self.secondary_type = line.start_type
                    add_line = True
                elif line.start_type == self.secondary_type:
                        add_line = True
            if line.end_type == Type.COMMENT_END:
                add_line = True
                self._done = True
        else:
            assert False
        if add_line:
            self.lines.append(line)
            return True
        else:
            return False

    def is_finished(self):
        return self._done

    def finish_or_die(self):
        if self.is_finished():
            return
        if self.primary_type == Type.DOUBLE_STAR:
            assert self._done, f"Needs termination: {self.lines[-1]}"
        else:
            self._done = True

    def __str__(self):
        return "\n".join(str(x) for x in self.lines)

    def __repr__(self):
        return f"<Docstring {self.lines[0].filename}:{self.lines[0].num}-{self.lines[-1].num}>"


def parse_docstrings(filename, raw_lines):
    # Add final line to fake an ending sentinel.
    raw_lines = raw_lines + ["\n"]
    docstrings = []
    docstring = None
    for num, raw_line in enumerate(raw_lines):
        line = Docline(filename, num, raw_line)
        added_line = False
        if docstring is None:
            if line.start_type in Type.PRIMARY_TYPES:
                added_line = True
                docstring = Docstring(line)
        else:
            added_line = docstring.maybe_add_next_line(line)
        needs_new_docstring = (not added_line or docstring.is_finished())
        if needs_new_docstring and docstring is not None:
            docstring.finish_or_die()
            docstrings.append(docstring)
            docstring = None
    assert docstring is None
    return docstrings


def main():
    filename = "common/cond.h"

    with open(filename, "r") as f:
        raw_lines = list(f.readlines())
        filename = f.name
    docstrings = parse_docstrings(filename, raw_lines)
    for docstring in docstrings:
        print(docstring)
        print("---")

assert  __name__ == "__main__"
main()
