#!/usr/bin/env python3

import argparse
from enum import Enum
import os
from os.path import abspath, dirname, join, basename
from textwrap import dedent, indent
from subprocess import run, STDOUT, PIPE
import sys

import numpy as np


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
        if text.startswith("///") and not text.startswith("////"):
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


def format_lines(lines, abbrev=False):
    num_width = len(str(lines[-1].num))
    text_lines = [x.format(num_width) for x in lines]
    if abbrev and len(lines) > 5:
        text_lines = [
            text_lines[0], text_lines[1], "...",
            text_lines[-2], text_lines[-1]]
    return "\n".join(text_lines)


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

    def to_text_lines(self):
        return [line.raw_line for line in self.lines]

    def assert_finished(self):
        pass

    def __str__(self):
        return format_lines(self.lines)

    def __len__(self):
        return len(self.lines)

    def __repr__(self):
        return (
            f"<{self.__class__.__name__} {self.lines[0].filename}:"
            f"{self.lines[0].num + 1}-{self.lines[-1].num + 1}>")


def chunk_cls_list():
    # N.B. Does not contain `GenericChunk`.
    return (
        # Docstrings.
        TripleSlashChunk,
        DoubleStarChunk,
        # Comments.
        DoubleSlashChunk,
        SingleStarChunk,
        # Whitespace.
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
        if line.start_token == Token.TRIPLE_SLASH:
            return super().add_line(line)
        else:
            raw = line.raw_line.strip()
            # Not really triple slash, but meh.
            if raw in ("//@{", "//@}"):
                new = f"/// {raw[2:]}"
                line = Docline(line.filename, line.num, f"{line.indent}{new}")
                return super().add_line(line)
            else:
                return False


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
                    lines_str = format_lines(self.lines + [line])
                    raise UserError(
                        f"Must continue with single star:\n"
                        f"{repr(line)}\n"
                        f"{lines_str}")
                do_add_line = True
        if not do_add_line:
            lines_str = format_lines(self.lines + [line])
            assert self._finished, (
                f"Needs termination:\n{lines_str}")
        if do_add_line:
            return super().add_line(line)


def make_new_chunk(line):
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
            active_chunk = make_new_chunk(line)
        elif not active_chunk.add_line(line):
            active_chunk.assert_finished()
            chunks.append(active_chunk)
            active_chunk = make_new_chunk(line)
        assert active_chunk is not None
    active_chunk.assert_finished()
    chunks.append(active_chunk)
    return chunks


def format_docstring(docstring):
    MAX_LEN = 80
    indent = docstring.lines[0].indent
    spacing = " "
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
            new_line = f"{indent}{spacing}{line}".rstrip()
            new_lines.append(new_line)
        last_line = text_lines[-1]
        new_lines += maybe_wrap(f"{spacing}{last_line}", " */")
    return new_lines


"""
ERROR: Docstring formatting is incorrect
./multibody/optimization/sliding_friction_complementarity_constraint.h:82:   /** Getter for the slack variable c, used in the constraint
./multibody/optimization/sliding_friction_complementarity_constraint.h:83: 

ERROR: Docstring formatting is incorrect
./multibody/tree/revolute_mobilizer.h:58:   /** @retval axis_F The rotation axis as a unit vector expressed in the inboard
./multibody/tree/revolute_mobilizer.h:59:                   frame F. */

ERROR: Docstring formatting is incorrect
./solvers/sdpa_free_format.h:97: /** SDPA format with free variables.
./solvers/sdpa_free_format.h:98: 

ERROR: Docstring formatting is incorrect
./systems/analysis/test_utilities/quadratic_scalar_system.h:11: /** System where the state at (scalar) time t corresponds to the quadratic
./systems/analysis/test_utilities/quadratic_scalar_system.h:12:   equation St² + St + 3, where S is a user-defined Scalar (4 by default). */


"""

def reformat_chunk(chunk):
    if isinstance(chunk, DocstringChunk):
        # Multi-pass for idempotent.
        # TODO(eric): Fix this.
        chunks = [chunk]
        first_line = chunk.lines[0]
        prev_lines = None
        for i in range(3):
            new_lines = format_docstring(chunk)
            if prev_lines is not None:
                if new_lines == prev_lines:
                    break
                else:
                    pass
            chunk = parse_single_chunk(
                new_lines, first_line.filename, first_line.num)
            chunks.append(chunk)
            prev_lines = new_lines
        else:
            for i, chunk in enumerate(chunks):
                print(f"i = {i}")
                print(chunk)
                print("---")
            assert False, "Bug in indempotent check"
        return new_lines
    else:
        return chunk.to_text_lines()


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
        text = "\n".join(format_docstring(docstring))
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


class Regex:
    class Part:
        def consume(self, xs, index):
            raise NotImplemented


    class Single:
        def __init__(self, check):
            self._check = check

        def __repr__(self):
            return f"<Single {self._check}>"

        def consume(self, xs, index):
            x = xs[index]
            if self._check(x):
                return [x]
            else:
                return None


    class Any:
        def __init__(self, check):
            self._check = check

        def __repr__(self):
            return f"<Any {self._check}>"

        def consume(self, xs, index):
            group = []
            while self._check(xs[index]) and index < len(xs):
                group.append(xs[index])
                index += 1
            return group


    def __init__(self, parts):
        self._parts = list(parts)

    def __repr__(self):
        return f"<Regex {self._parts}>"

    def find_all(self, xs):
        matches = []
        index = 0
        while index < len(xs):
            start_index = index
            match = []
            for part in self._parts:
                if index == len(xs):
                    match = None
                    break
                group = part.consume(xs, index)
                if group is not None:
                    index += len(group)
                    match.append(group)
                else:
                    match = None
                    break
            if match is not None:
                assert index > start_index
                matches.append(match)
            else:
                # Matching failed. Restart.
                index = start_index + 1
        return matches


def is_whitespace(chunk):
    return isinstance(chunk, WhitespaceChunk)


def is_meaningful_docstring(chunk):
    if isinstance(chunk, DocstringChunk):
        if len(chunk.lines) == 1:
            if chunk.lines[0].text.strip().startswith("@"):
                return False
        return True
    return False


def is_generic_but_not_macro(chunk):
    if isinstance(chunk, GenericChunk):
        if not chunk.lines[0].text.startswith("#"):
            return True
    return False


def is_comment_but_not_nolint(chunk):
    if isinstance(chunk, CommentChunk):
        # TODO(eric.cousineau): Figure out how to make cpplint play nicely with
        # mkdoc.py?
        if chunk.lines[-1].text.strip().startswith("NOLINTNEXTLINE"):
            return False
        return True
    return False


class LintErrors:
    class Item:
        def __init__(self, text, lines):
            self.text = text
            self.lines = lines

    def __init__(self):
        self.items = []

    def add(self, text, lines):
        self.items.append(self.Item(text, lines))


def print_chunks(chunks):
    for chunk in chunks:
        print(chunk)


def reorder_chunks(chunks, lint_errors):
    mkdoc_issue = Regex([
        Regex.Single(is_meaningful_docstring),
        Regex.Any(is_whitespace),
        Regex.Single(is_comment_but_not_nolint),
        Regex.Single(is_generic_but_not_macro),
    ])
    matches = mkdoc_issue.find_all(chunks)
    for match in matches:
        (doc,), ws, (comment,), (generic,) = match
        if lint_errors is not None:
            lint_errors.add(
                "ERROR: Docstring must be placed directly next to symbol for "
                "mkdoc.py",
                lines=[doc.lines[-1], comment.lines[0], generic.lines[0]],
            )
        else:
            original = [doc] + ws + [comment, generic]
            start = chunks.index(original[0])
            for x in original:
                chunks.remove(x)
            comment.lines = [
                x for x in comment.lines if x.text.strip() != ""]
            new = [comment] + ws + [doc, generic]
            for i, x in enumerate(new):
                chunks.insert(start + i, x)
    return chunks


def parse_single_chunk(new_lines, filename, start_num):
    chunk = None
    for i, new_line in enumerate(new_lines):
        fake_line = Docline(filename, start_num + i, new_line)
        if chunk is None:
            chunk = make_new_chunk(fake_line)
        else:
            assert chunk.add_line(fake_line), fake_line
    chunk.assert_finished()
    return chunk


def lint_chunk(lint_errors, chunk, new_lines):
    if lint_errors is None:
        return
    first_line = chunk.lines[0]
    # Reparse to ensure that our new chunk is still valid.
    new_chunk = parse_single_chunk(
        new_lines, first_line.filename, first_line.num)
    # Compare.
    if chunk.to_text_lines() != new_chunk.to_text_lines():
        lint_errors.add(
            "ERROR: Docstring formatting is incorrect",
            lines=chunk.lines[:2],
        )


def transform(filename, lint):
    with open(filename, "r") as f:
        raw_lines = [x.rstrip() for x in f.readlines()]
    chunks = parse_chunks(filename, raw_lines)
    if lint:
        lint_errors = LintErrors()
    else:
        lint_errors = None
    chunks = reorder_chunks(chunks, lint_errors)
    # Replace docstrings with "re-rendered" version.
    new_lines = []
    for chunk in chunks:
        new_lines_i = reformat_chunk(chunk)
        lint_chunk(lint_errors, chunk, new_lines_i)
        new_lines += new_lines_i
    if lint:
        errors = lint_errors.items
        if len(errors) == 0:
            return True
        errors = sorted(errors, key=lambda x: (x.lines[0].num, x.text))
        for i, error in enumerate(errors):
            if i == 3:
                remaining = len(errors) - 3
                print(f"ERROR: There are {remaining} more errors for: {filename}")
                print()
                break
            print(error.text)
            print(format_lines(error.lines))
            print()
        return False
    else:
        with open(filename, "w") as f:
            f.write("\n".join(new_lines))
            f.write("\n")
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("filenames", type=str, nargs="*")
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--fix", action="store_true")
    args = parser.parse_args()

    filenames = args.filenames

    if args.all:
        assert len(filenames) == 0
        source_tree = join(dirname(__file__), "../..")
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

    good = True
    for filename in filenames:
        if not transform(filename, lint=not args.fix):
            good = False
    if not good:
        sys.exit(1)


assert  __name__ == "__main__"
main()
