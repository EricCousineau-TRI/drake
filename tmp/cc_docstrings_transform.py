#!/usr/bin/env python3

import argparse
from enum import Enum
from os.path import abspath, dirname, join, basename
from subprocess import run, STDOUT, PIPE


class Type:
    OTHER = ""
    TRIPLE_SLASH = "///"
    DOUBLE_STAR = "/**"
    SINGLE_STAR = "* "
    COMMENT_END = "*/"

    # TODO(eric): Some lines are just `//@{` and `//@}`... should all sections
    # be like that?

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


class Chunk:
    def __init__(self, line):
        assert isinstance(line, Docline)
        self.lines = [line]

    def add_line(self, line):
        assert line.num == self.lines[-1] + 1
        self.lines.append(line)

    def __str__(self):
        return "\n".join(str(x) for x in self.lines)

    def __repr__(self):
        return (
            f"<{self.__class__} {self.lines[0].filename}:"
            f"{self.lines[0].num}-{self.lines[-1].num}>")


class Docstring(Chunk):
    def __init__(self, line):
        self.primary_type = line.start_type
        self.secondary_type = Type.OTHER
        self._done = False
        super().__init__(line)

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
            self.add_line(line)
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


def reformat_docstring(raw_lines, docstring):
    lines = docstring.lines
    first_line_num = lines[0].num
    last_line_num = lines[-1].num
    first_line = lines[0]
    while lines[-1].text.strip() == "":
        lines = lines[:-1]
    indent = first_line.indent
    # Wrapping?
    if len(lines) == 1:
        new_lines = [f"{indent}/** {first_line.text.lstrip()} */\n"]
    else:
        new_lines = [f"{indent}/** {first_line.text.lstrip()}\n"]
        for line in lines[1:]:
            new_line = f"{indent}{line.text}".rstrip()
            new_lines.append(f"{new_line}\n")
        new_lines.append(f"{indent} */\n")
    # Replace lines.
    del raw_lines[first_line_num:last_line_num + 1]
    for i, new_line in enumerate(new_lines):
        raw_lines.insert(first_line_num + i, new_line)


def parse_chunks(filename, raw_lines):
    chunks = []
    chunk = None
    docstring = None
    for num, raw_line in enumerate(raw_lines):
        line = Docline(filename, num, raw_line)

        if docstring is None:
            is_docstring_line = False
            if line.start_type in Type.PRIMARY_TYPES:
                is_docstring_line = True
                docstring = Docstring(line)
        else:
            is_docstring_line = docstring.maybe_add_next_line(line)

        if is_docstring_line:
            complete_docstring = docstring.is_finished()
            complete_chunk = (chunk is not None)
            if chunk is None:
                chunk = Chunk(line)
            else:
                chunk.add_line(line)
        else:
            complete_docstring = (docstring is not None)
            complete_chunk = False

        if complete_docstring:
            docstring.finish_or_die()
            chunks.append(docstring)
            docstring = None
        if complete_chunk:
            chunks.append(chunk)
            chunk = None

        # We should always have either a chunk or a docstring.
        assert chunk is not None or docstring is not None

    # All docstrings should be finished.
    if docstring is not None:
        docstring.finish_or_die()
        chunks.append(docstring)
    if chunk is not None:
        chunks.append(chunk)
    return chunks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", type=str)
    args = parser.parse_args()
    filename = args.filename

    with open(filename, "r") as f:
        raw_lines = list(f.readlines())
    docstrings = parse_docstrings(filename, raw_lines)
    # Replace docstrings with "re-rendered" version.
    new_lines = []
    for docstring in docstrings:
        reformat_docstring(raw_lines, docstring)
    with open(filename, "w") as f:
        for line in raw_lines:
            f.write(line)


assert  __name__ == "__main__"
main()
