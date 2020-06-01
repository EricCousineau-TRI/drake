#!/usr/bin/env python3

import argparse
from enum import Enum
import os
from os.path import abspath, dirname, join, basename
from textwrap import dedent, indent
from subprocess import run, STDOUT, PIPE


class Token:
    NOTHING = ""
    TRIPLE_SLASH = "///"
    DOUBLE_SLASH = "//"
    SLASH_DOUBLE_STAR = "/**"
    SLASH_SINGLE_STAR = "/*"
    SINGLE_STAR = "*"
    STAR_SLASH_END = "*/"


def parse_line_tokens(text):
    start_token = Token.NOTHING
    end_token = Token.NOTHING
    if len(text) > 0:
        # Text must already be stripped..
        assert text[0] != " ", repr(text)
        assert text[-1] != " "
        if text.startswith("///"):
            start_token = Token.TRIPLE_SLASH
        elif text.startswith("//"):
            start_token = Token.DOUBLE_SLASH
        elif text.startswith("/**"):
            start_token = Token.SLASH_DOUBLE_STAR
        elif text.startswith("/*"):
            start_token = Token.SLASH_SINGLE_STAR
        elif text.startswith("* ") or text.strip() == "*":
            start_token = Token.SINGLE_STAR
        has_start_token_for_end = (
            start_token in (
                Token.NOTHING, Token.SINGLE_STAR,
                Token.SLASH_SINGLE_STAR, Token.SLASH_DOUBLE_STAR))
        if text.endswith("*/") and has_start_token_for_end:
            end_token = Token.STAR_SLASH_END
    return start_token, end_token


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
        self.start_token, self.end_token = parse_line_tokens(without_indent)
        self._set_text(f"{self.indent}{self.start_token}")

    def _set_text(self, prefix):
        if self.raw_line.strip() == "":
            self.text = ""
            return
        assert self.raw_line.startswith(prefix), (
            repr(self.raw_line), repr(prefix))
        self.text = self.raw_line[len(prefix):]
        if self.end_token == Token.STAR_SLASH_END:
            self.text = self.text[:-len(self.end_token)].rstrip("*").rstrip()

    def reset_indent(self, indent):
        self.indent = indent
        self._set_text(self.indent)

    def __repr__(self):
        return (
            f"<{self.__class__.__name__} "
            f"{self.filename}:{self.num + 1} "
            f"start_token={repr(self.start_token)} "
            f"end_token={repr(self.end_token)}"
            f">")

    def __str__(self):
        return self.format()

    def format(self, num_width=None):
        if num_width == None:
            num_width = len(str(self.num))
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

    def assert_finished(self):
        pass

    def __str__(self):
        num_width = len(str(self.lines[-1].num))
        return "\n".join(x.format(num_width) for x in self.lines)

    def __len__(self):
        return len(self.lines)

    def __repr__(self):
        return (
            f"<{self.__class__} {self.lines[0].filename}:"
            f"{self.lines[0].num + 1}-{self.lines[-1].num + 1}>")


def chunk_cls_list():
    # N.B. Does not contain `GenericChunk`.
    return (
        TripleSlashChunk,
        DoubleStarChunk,
        DoubleSlashChunk,
        SingleStarChunk,
        WhitespaceChunk,
    )


class GenericChunk(Chunk):
    def add_line(self, line):
        for cls in chunk_cls_list():
            if cls().add_line(line):
                return False
        return super().add_line(line)


class WhitespaceChunk(Chunk):
    def add_line(self, line):
        if line.raw_line.strip() == "":
            return super().add_line(line)
        else:
            return False


class UserError(RuntimeError):
    pass


def _dedent_lines(lines, chunk, require_nonragged):
    # Dedent all following lines.
    text = dedent("\n".join(lines))
    if require_nonragged and len(text) > 0:
        # Ensure that first nonempty line does not start with whitespace
        # (ragged indentation?).
        if text.lstrip("\n")[0] == " ":
            raise UserError(f"Must not have ragged indentation:\n{chunk}")
    return text


def _remove_empty_leading_trailing_lines(lines):
    # Remove empty leading and trailing lines.
    while lines[0].strip() == "":
        del lines[0]
    while lines[-1].strip() == "":
        del lines[-1]


class CommentChunk(Chunk):
    pass


class DoubleSlashChunk(CommentChunk):
    def add_line(self, line):
        if line.start_token != Token.DOUBLE_SLASH:
            return False
        return super().add_line(line)


class SingleStarChunk(CommentChunk):
    def __init__(self):
        self._finished = False
        super().__init__()

    def add_line(self, line):
        if len(self.lines) == 0 and line.start_token != Token.SLASH_SINGLE_STAR:
            return False
        if self._finished:
            return False
        if line.end_token == Token.STAR_SLASH_END:
            self._finished = True
        return super().add_line(line)

    def assert_finished(self):
        assert self._finished, f"Not closed:\n{self}"


class DocstringChunk(Chunk):
    def get_docstring_text(self):
        text_lines = [line.text for line in self.lines]
        _remove_empty_leading_trailing_lines(text_lines)
        return _dedent_lines(text_lines, self, require_nonragged=True).strip()


class TripleSlashChunk(DocstringChunk):
    def add_line(self, line):
        if line.start_token != Token.TRIPLE_SLASH:
            return False
        return super().add_line(line)


class DoubleStarChunk(DocstringChunk):
    def __init__(self):
        self.secondary_type = None
        self._finished = False
        super().__init__()

    def assert_finished(self):
        assert self._finished, f"Not closed:\n{self}"

    def get_docstring_text(self):
        if self.lines[0].text == "":
            return super().get_docstring_text()
        else:
            text_lines = [line.text for line in self.lines]
            _remove_empty_leading_trailing_lines(text_lines)
            text = text_lines[0].lstrip() + "\n"
            text += _dedent_lines(text_lines[1:], self, require_nonragged=False)
            return text.strip()

    def add_line(self, line):
        if len(self.lines) == 0 and line.start_token != Token.SLASH_DOUBLE_STAR:
            return False
        if self._finished:
            return False

        def lines_str():
            tmp = Chunk()
            tmp.lines = self.lines + [line]
            return str(tmp)

        do_add_line = False

        if line.end_token == Token.STAR_SLASH_END:
            self._finished = True
            do_add_line = True

        if len(self.lines) == 0:
            do_add_line = True
        else:
            if self.secondary_type is None:
                if line.start_token == Token.SINGLE_STAR:
                    self.secondary_type = Token.SINGLE_STAR
                else:
                    self.secondary_type = Token.NOTHING
            if self.secondary_type == Token.NOTHING:
                do_add_line = True
                # Reset indentation to match first line.
                line.reset_indent(self.lines[0].indent)
            else:
                if not do_add_line and line.start_token != Token.SINGLE_STAR:
                    raise UserError(
                        f"Must continue with single star:\n"
                        f"{repr(line)}\n"
                        f"{lines_str()}")
                do_add_line = True
        if not do_add_line:
            assert self._finished, (
                f"Needs termination:\n{lines_str()}")
        if do_add_line:
            return super().add_line(line)


def new_chunk(line):
    for cls in chunk_cls_list():
        chunk = cls()
        if chunk.add_line(line):
            return chunk
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
            active_chunk.assert_finished()
            chunks.append(active_chunk)
            active_chunk = new_chunk(line)
        assert active_chunk is not None
    active_chunk.assert_finished()
    chunks.append(active_chunk)
    return chunks


def format_docstring(docstring):
    MAX_LEN = 80
    indent = docstring.lines[0].indent
    text = docstring.get_docstring_text()
    # Can't have nested comments :(
    # Replace with magical D-style stuff.
    text = text.replace("*/", "+/").replace("/*", "/+")
    text_lines = text.split("\n")
    first_line = text_lines[0]

    def maybe_wrap(text, suffix):
        new_line = f"{indent}{text}"
        too_long = len(new_line) + len(suffix) > MAX_LEN
        should_extend = ("@endcode" in text or "</pre>" in text)
        if too_long or should_extend:
            return [
                new_line,
                f"{indent}{suffix}",
            ]
        else:
            return [f"{new_line}{suffix}"]

    if len(text_lines) == 1:
        if "://" in first_line:
            # Weird behavior with bogus lint?
            return [f"{indent}/// {first_line}"]
        new_lines = maybe_wrap(f"/** {first_line}", " */")
    else:
        new_lines = [f"{indent}/** {first_line}"]
        for line in text_lines[1:-1]:
            new_line = f"{indent} {line}".rstrip()
            new_lines.append(new_line)
        last_line = text_lines[-1]
        new_lines += maybe_wrap(f" {last_line}", " */")
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
    assert new_lines == ["/** Hello /+ world +/ */"]
    print("\n".join(new_lines))

    yar = dedent("""\
        /**
         * Something
         *    with code
         *
         * Don't you see?
         */
    """.rstrip())
    docstring, = parse_chunks("test", yar.split("\n"))
    new_lines = reformat_chunk(docstring)
    # Enusre this works...
    text = "\n".join(new_lines)
    print(text)

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
    except UserError as e:
        assert "ragged indentation" in str(e)
        print(str(e))


def reorder_chunks(chunks):
    prev_chunk = None
    for chunk in chunks:
        if isinstance(chunk, CommentChunk) and isinstance(prev_chunk, DocstringChunk):
            print(prev_chunk)
            print(chunk)
            print("---")
        prev_chunk = chunk
    return chunks


def transform(filename, dry_run):
    with open(filename, "r") as f:
        raw_lines = [x.rstrip() for x in f.readlines()]
    chunks = parse_chunks(filename, raw_lines)
    chunks = reorder_chunks(chunks)
    # Replace docstrings with "re-rendered" version.
    new_lines = []
    for chunk in chunks:
        new_lines += reformat_chunk(chunk)
    if dry_run:
        return
    with open(filename, "w") as f:
        f.write("\n".join(new_lines))
        f.write("\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("filenames", type=str, nargs="*")
    parser.add_argument("--all", action="store_true")
    parser.add_argument("-n", "--dry_run", action="store_true")
    args = parser.parse_args()

    filenames = args.filenames

    if args.all:
        assert len(filenames) == 0
        source_tree = abspath(join(dirname(__file__), ".."))
        os.chdir(source_tree)
        result = run(
            ["find", ".", "-name", "*.h"], check=True, stdout=PIPE, encoding="utf8")
        filenames = result.stdout.strip().split("\n")
        filenames.sort()
        for filename in list(filenames):
            if filename.startswith(("./attic", "./third_party", "./tools")):
                filenames.remove(filename)

    if filenames == ["<test>"]:
        assert not args.dry_run
        test()
        return

    for filename in filenames:
        try:
            transform(filename, dry_run=args.dry_run)
            if not args.dry_run:
                # Run it once more (for "idempotent" check...).
                transform(filename, dry_run=False)
        except UserError as e:
            print(indent(str(e), prefix="  "))


assert  __name__ == "__main__"
main()
