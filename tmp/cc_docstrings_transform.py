#!/usr/bin/env python3

import argparse
from enum import Enum
from os.path import abspath, dirname, join, basename
from subprocess import run, STDOUT, PIPE


class Type:
    NOTHING = ""
    TRIPLE_SLASH = "///"
    DOUBLE_STAR = "/**"
    SINGLE_STAR = "* "
    COMMENT_END = "*/"

    # TODO(eric): Some lines are just `//@{` and `//@}`... should all sections
    # be like that?

    PRIMARY_TYPES = (TRIPLE_SLASH, DOUBLE_STAR)
    SECONDARY_TYPES = (NOTHING, SINGLE_STAR)

    @staticmethod
    def parse(text):
        start_type = Type.NOTHING
        end_type = Type.NOTHING
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
        self.text = without_indent[len(self.start_type):]
        if self.end_type == Type.COMMENT_END:
            self.text = self.text[:-len(self.end_type)].rstrip("*")

    def reset_indent(self, indent):
        assert self.raw_line.startswith(indent)
        self.text = self.raw_line[len(indent):]
        if self.end_type == Type.COMMENT_END:
            self.text = self.text[:-len(self.end_type)].rstrip("*")

    def __repr__(self):
        return f"<{self.__class__.__name__} {self.filename}:{self.num + 1}>"

    def __str__(self):
        return f"{self.filename}:{self.num + 1:<5}: {self.raw_line}"


class Chunk:
    def __init__(self):
        self.lines = []

    def add_line(self, line):
        assert isinstance(line, Docline)
        if len(self.lines) > 0:
            if line.num != self.lines[-1].num + 1:
                print(self.lines[-1])
                print(line)
                assert False
        self.lines.append(line)
        return True

    def __str__(self):
        return "\n".join(str(x) for x in self.lines)

    def __len__(self):
        return len(self.lines)

    def __repr__(self):
        return (
            f"<{self.__class__} {self.lines[0].filename}:"
            f"{self.lines[0].num + 1}-{self.lines[-1].num + 1}>")

    def finish_or_die(self):
        raise NotImplemented


class GenericChunk(Chunk):
    def add_line(self, line):
        if line.start_type in Type.PRIMARY_TYPES:
            return False
        else:
            return super().add_line(line)

    def finish_or_die(self):
        pass


class DocstringChunk(Chunk):
    def __init__(self):
        self.primary_type = None
        self.secondary_type = None
        self._finished = False
        super().__init__()

    def add_line(self, line):
        if self._finished:
            return False
        is_first_line = False

        if self.primary_type is None:
            if line.start_type not in Type.PRIMARY_TYPES:
                return False
            is_first_line = True
            self.primary_type = line.start_type

        do_add_line = False
        if self.primary_type == Type.TRIPLE_SLASH:
            if line.start_type == Type.TRIPLE_SLASH:
                do_add_line = True
            else:
                self._finished = True
        elif self.primary_type == Type.DOUBLE_STAR:
            if is_first_line:
                do_add_line = True
            elif line.start_type in Type.SECONDARY_TYPES:
                if self.secondary_type is None:
                    # Use new secondary type.
                    self.secondary_type = line.start_type
                    do_add_line = True
                elif line.start_type == self.secondary_type:
                    # Require secondary type.
                    do_add_line = True
                if do_add_line and line.start_type == Type.NOTHING:
                    # Readjust indentation to match first line.
                    line.reset_indent(self.lines[0].indent)
            if line.end_type == Type.COMMENT_END:
                do_add_line = True
                self._finished = True
        if do_add_line:
            return super().add_line(line)
        else:
            return False

    def finish_or_die(self):
        if self._finished:
            return
        if self.primary_type == Type.DOUBLE_STAR:
            assert self._finished, f"Needs termination:\n{self}"
        else:
            self._finished = True


def parse_chunks(filename, raw_lines):

    def finish_chunk(chunk):
        chunk.finish_or_die()
        if len(chunk) > 0:
            chunks.append(chunk)

    def next_chunk(chunk):
        if isinstance(chunk, GenericChunk):
            return DocstringChunk()
        else:
            return GenericChunk()

    chunks = []
    active_chunk = GenericChunk()
    for num, raw_line in enumerate(raw_lines):
        line = Docline(filename, num, raw_line)
        possible_chunks = [
            lambda: active_chunk,
            lambda: next_chunk(active_chunk),
        ]
        for chunk_func in possible_chunks:
            chunk = chunk_func()
            if chunk.add_line(line):
                active_chunk = chunk
                break
            else:
                finish_chunk(chunk)
        else:
            assert False
    finish_chunk(active_chunk)
    return chunks


def reformat_docstring(docstring):

    def sanitize(text):
        # Can't have nested comments :(
        text = text.replace("*/", "* /")
        return text.rstrip()

    lines = docstring.lines
    first_line = lines[0]
    while lines[-1].text.strip() == "":
        lines = lines[:-1]
    indent = first_line.indent
    # Wrapping?
    first_line_text = sanitize(first_line.text.lstrip())
    if len(lines) == 1:
        new_lines = [f"{indent}/** {first_line_text} */\n"]
    else:
        new_lines = [f"{indent}/** {first_line_text}\n"]
        for line in lines[1:-1]:
            text = sanitize(line.text)
            if text:
                new_lines.append(f"{indent}{text}\n")
            else:
                new_lines.append("\n")
        last_line = lines[-1]
        new_lines.append(f"{indent}{sanitize(last_line.text)}  */\n")
    return new_lines


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", type=str)
    args = parser.parse_args()
    filename = args.filename

    with open(filename, "r") as f:
        raw_lines = list(f.readlines())
    chunks = parse_chunks(filename, raw_lines)
    # Replace docstrings with "re-rendered" version.
    new_lines = []
    for chunk in chunks:
        if isinstance(chunk, DocstringChunk):
            new_lines += reformat_docstring(chunk)
        else:
            new_lines += [x.raw_line + "\n" for x in chunk.lines]
    with open(filename, "w") as f:
        for line in new_lines:
            f.write(line)


assert  __name__ == "__main__"
main()
