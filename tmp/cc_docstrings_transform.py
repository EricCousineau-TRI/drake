#!/usr/bin/env python3

from enum import Enum
from os.path import abspath, dirname, join, basename
from subprocess import run, STDOUT, PIPE


class Type(Enum):
    OTHER = ""
    TRIPLE_SLASH = "///"
    DOUBLE_STAR = "/**"
    SINGLE_STAR = "* "
    COMMENT_END = "*/"

    PRIMARY_TYPES = (TRIPLE_SLASH, DOUBLE_START)
    SECONDARY_TYPES = (OTHER, SINGLE_STAR)

    @staticmethod
    def parse(text):
        start_type = Type.OTHER
        end_type = Type.OTHER
        if len(text) > 0:
            # Text must already be stripped..
            assert text[0] != " "
            assert text[-1] != " "
            if text.startswith("///"):
                start_type = Type.TRIPLE_SLASH
            elif text.startswith("/**"):
                start_type = Type.DOUBLE_STAR_ONLY
            elif text.startswith("* "):
                start_type = Type.SINGLE_STAR
            if text.endswith("*/"):
                end_type = Type.COMMENT_END
        return start_type, end_type


def get_indent(line):
    for i in range(len(line)):
        assert c != '\t'
        if c != ' ':
            break
    return line[:i], line[i:]


class Docline:
    def __init__(self, filename, num, raw_line):
        self.filename = filename
        self.num = num
        without_indent, self.indent = split_indent(raw_line)
        self.start_type, self.end_type = Type.parse(without_indent)
        self.text = None
        if self.start_type != Type.OTHER:
            self.text = without_indent[len(self.start_type):]
            if self.end_type == Type.COMMENT_END:
                self.text = self.text[:-len("*/")].rstrip("*")

    def __repr__(self):
        return f"<Docline {basename(self.filename)}:{self.num}>"


class Docstring:
    def __init__(self, line):
        assert isinstance(line, Docline)
        self.lines = [line]
        self.primary_type = line.start_type
        self.secondary_type = Type.OTHER
        self.done = False

    def maybe_add_next_line(self, line):
        add_line = False
        require_done_if_not_added = False
        if self.lines[0].type == Type.TRIPLE_SLASH:
            if line.start_type == Type.TRIPLE_SLASH:
                add_line = True
            else:
                self.done = True
        elif self.lines[0].type == Type.DOUBLE_STAR:
            require_done_if_not_added = True
            if line.start_type in TYPE.SECONDARY_TYPES:
                if len(self.lines) == 1:
                    self.secondary_type = line.start_type
                    add_line = True
                elif line.start_type == self.secondary_type:
                        add_line = True
            if line.end_type == Type.COMMENT_END:
                add_line = True
                self.done = True
        if add_line:
            self.lines.append(line)
            return True
        else:
            if require_done_if_not_added:
                assert self.done, line
            return False


def parse_docstrings(f):
    raw_lines = list(f.readlines())
    filename = abspath(f.name)
    docstrings = []
    docstring = None
    for num, raw_line in enumerate(raw_lines):
        line = Docline(num, raw_line)
        if docstring is None:
            if line.start_type in Type.PRIMARY_TYPES:
                docstring = Docstring(line)
        else:
            docstring.maybe_add_next_line(line)
            if docstring.done():
                docstrings.append(docstring)
                docstring = None
    if docstring is not None:
        end_line = Docline(-1, "")
        docstring.maybe_add_next_line(end_line)
        # File has docstring syntax error?
        assert docstring.done(), filename
        docstrings.append(docstring)
    return docstrings


def main():
    filename = 


assert  __name__ == "__main__"
main()
