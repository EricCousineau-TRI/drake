#!/usr/bin/env python3

import argparse
from enum import Enum
from os.path import abspath, dirname, join, basename
from textwrap import dedent, indent
from subprocess import run, STDOUT, PIPE


class Type:
    NOTHING = ""
    TRIPLE_SLASH = "///"
    DOUBLE_STAR = "/**"
    SINGLE_STAR = "*"
    COMMENT_END = "*/"

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
            elif text.startswith("*"):
                start_type = Type.SINGLE_STAR
            possible_end = (Type.NOTHING, Type.SINGLE_STAR, Type.DOUBLE_STAR)
            if start_type in possible_end and text.endswith("*/"):
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
        assert "\n" not in raw_line
        self.raw_line = raw_line
        self.indent, without_indent = split_indent(self.raw_line)
        self.start_type, self.end_type = Type.parse(without_indent)
        self._set_text(f"{self.indent}{self.start_type}")

    def _set_text(self, prefix):
        if self.raw_line.strip() == "":
            self.text = ""
            return
        assert self.raw_line.startswith(prefix), (
            repr(self.raw_line), repr(prefix))
        self.text = self.raw_line[len(prefix):]
        if self.end_type == Type.COMMENT_END:
            self.text = self.text[:-len(self.end_type)].rstrip("*").rstrip()

    def reset_indent(self, indent):
        self.indent = indent
        self._set_text(self.indent)

    def __repr__(self):
        return f"<{self.__class__.__name__} {self.filename}:{self.num + 1}>"

    def __str__(self):
        return self.format(num_width=3)

    def format(self, num_width):
        return f"{self.filename}:{self.num + 1:<{num_width}}: {self.raw_line}"


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
        num_width = len(str(self.lines[-1].num))
        return "\n".join(x.format(num_width) for x in self.lines)

    def __len__(self):
        return len(self.lines)

    def __repr__(self):
        return (
            f"<{self.__class__} {self.lines[0].filename}:"
            f"{self.lines[0].num + 1}-{self.lines[-1].num + 1}>")


class GenericChunk(Chunk):
    def add_line(self, line):
        if line.start_type in (Type.TRIPLE_SLASH, Type.DOUBLE_STAR):
            return False
        else:
            return super().add_line(line)


class DocstringChunk(Chunk):
    def get_docstring_text(self):
        text_lines = [line.text for line in self.lines]
        # Remove empty leading and trailing lines.
        while text_lines[0].strip() == "":
            del text_lines[0]
        while text_lines[-1].strip() == "":
            del text_lines[-1]
        # Ensure first line has no space.
        text = text_lines[0].lstrip() + "\n"
        # Dedent all following lines.
        final_part = dedent("\n".join(text_lines[1:]))
        if len(final_part) > 0:
            # Ensure that first nonempty line does not start with whitespace
            # (ragged indentation).
            if final_part.lstrip("\n")[0] == " ":
                assert False, f"Must not have ragged indentation:\n{self}"
        text += final_part
        return text.strip()


class TripleSlashChunk(DocstringChunk):
    def add_line(self, line):
        if line.start_type != Type.TRIPLE_SLASH:
            return False
        return super().add_line(line)


class DoubleStarChunk(DocstringChunk):
    def __init__(self):
        self.secondary_type = None
        self._finished = False
        super().__init__()

    def add_line(self, line):
        if self._finished:
            return False
        do_add_line = False
        if len(self.lines) == 0:
            if line.start_type != Type.DOUBLE_STAR:
                return False
            else:
                do_add_line = True
        elif line.start_type in (Type.NOTHING, Type.SINGLE_STAR):
            secondary_matches = (line.start_type == self.secondary_type)
            if self.secondary_type is None or secondary_matches:
                self.secondary_type = line.start_type
                do_add_line = True
                if line.start_type == Type.NOTHING:
                    # Reset indentation to match first line.
                    line.reset_indent(self.lines[0].indent)
        if line.end_type == Type.COMMENT_END:
            self._finished = True
            do_add_line = True
        if not do_add_line:
            tmp = Chunk()
            tmp.lines = self.lines + [line]
            for line in tmp.lines:
                print(f"{(line.start_type, line.end_type)}: {line}")
            assert self._finished, f"Needs termination:\n{tmp}"
        if do_add_line:
            return super().add_line(line)


def new_chunk(line):
    if line.start_type == Type.TRIPLE_SLASH:
        chunk = TripleSlashChunk()
    elif line.start_type == Type.DOUBLE_STAR:
        chunk = DoubleStarChunk()
    else:
        chunk = GenericChunk()
    assert chunk.add_line(line), line
    return chunk


def parse_chunks(filename, raw_lines):
    chunks = []
    active_chunk = None
    for num, raw_line in enumerate(raw_lines):
        line = Docline(filename, num, raw_line)
        if active_chunk is None:
            active_chunk = new_chunk(line)
        elif not active_chunk.add_line(line):
            chunks.append(active_chunk)
            active_chunk = new_chunk(line)
        assert active_chunk is not None
    chunks.append(active_chunk)
    return chunks


def format_docstring(docstring):
    indent = docstring.lines[0].indent
    text = docstring.get_docstring_text()
    # Can't have nested comments :(
    text = text.replace("*/", "* /")
    text_lines = text.split("\n")
    first_line = text_lines[0]
    if len(text_lines) == 1:
        new_lines = [f"{indent}/** {first_line} */"]
    else:
        new_lines = [f"{indent}/** {first_line}"]
        for line in text_lines[1:-1]:
            if line:
                new_lines.append(f"{indent} {line}")
            else:
                new_lines.append("\n")
        last_line = text_lines[-1]
        new_lines.append(f"{indent} {last_line} */")
    return new_lines


def reformat_chunk(chunk):
    if isinstance(chunk, DocstringChunk):
        new_lines = format_docstring(chunk)
    else:
        new_lines = []
        for line in chunk.lines:
            if line.text == "//@{":
                new_line = f"{line.indent}/** @{{ */"
            elif line.text == "//@}":
                new_line = f"{line.indent}/** @}} */"
            else:
                new_line = line.raw_line
            new_lines.append(new_line)
    return new_lines


def test():
    block = dedent("""\
        /** abc

         def
           ghi

         jkl */

        /**     abc

         def
           ghi

         jkl */

        /** abc

            def
              ghi

            jkl
                **/

        /// abc
        ///
        /// def
        ///   ghi
        ///
        /// jkl

        /**
         *  abc
         *
         *  def
         *    ghi
         *
         *  jkl
         **/
    """.rstrip())
    chunks = parse_chunks("test", block.split("\n"))
    texts = []
    for docstring in chunks:
        if not isinstance(docstring, DocstringChunk):
            continue
        # print(docstring)
        text = "\n".join(reformat_chunk(docstring))
        print(text)
        texts.append(text)
        print("---")
    assert len(texts) == 5, len(texts)
    for text in texts[1:]:
        assert text == texts[0]

    maybe = ["/// Hello /* world */"]
    docstring, = parse_chunks("test", maybe)
    new_lines = reformat_chunk(docstring)
    print("\n".join(new_lines))

    ragged = dedent("""\
        /// abc
        /// def
        ///ghe
    """.rstrip())

    chunk, = parse_chunks("test", ragged.split("\n"))
    # Ragged indent.
    try:
        print(chunk.get_docstring_text())
        assert False
    except AssertionError as e:
        assert "ragged indentation" in str(e)
        print(str(e))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", type=str)
    args = parser.parse_args()
    filename = args.filename

    if filename == "<test>":
        test()
        return

    with open(filename, "r") as f:
        raw_lines = [x.rstrip() for x in f.readlines()]
    chunks = parse_chunks(filename, raw_lines)
    # Replace docstrings with "re-rendered" version.
    new_lines = []
    for chunk in chunks:
        new_lines += reformat_chunk(chunk)
    with open(filename, "w") as f:
        f.write("\n".join(new_lines))
        f.write("\n")


assert  __name__ == "__main__"
main()
