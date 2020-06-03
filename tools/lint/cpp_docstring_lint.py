#!/usr/bin/env python3

"""
Implements a simple parser which:

- Can report lint errors if a canonical docstring format is not adhered to.
- Can report lint errors if docstrings and comments being intermixed may
  confused `mkdoc.py`.
- Tokenizes a file based on lines, specifically geared towards docstrings.
  This is filled with heuristics, but can lint / reformat the codebase quickly.
- Can fix all docstrings to a standard overall form, while trying to maintain
  the docstring formatting (e.g. indentations, markdown, etc).
"""

import argparse
import os
from textwrap import dedent
from subprocess import run, PIPE
import sys

import numpy as np


def parse_line_tokens(text):
    """Parses the start and end "tokens" for a line. These tokens are meant to
    deal with comments.

    The input `text` should not start or end with any whitespace."""
    start_token = ""
    end_token = ""
    if len(text) > 0:
        # Text must already be stripped..
        assert text[0] != " ", repr(text)
        assert text[-1] != " "
        if text.startswith("///") and not text.startswith("////"):
            start_token = "///"
        elif text.startswith("//"):
            start_token = "//"
        elif text.startswith("/**"):
            start_token = "/**"
        elif text.startswith("/*"):
            start_token = "/*"
        elif text.startswith("* ") or text.strip() == "*":
            start_token = "*"
        has_start_token_for_end = (start_token in ("", "*", "/*", "/**"))
        if text.endswith("*/") and has_start_token_for_end:
            end_token = "*/"
    return start_token, end_token


class FileLine:
    """Indicates a line in a file with a given number."""
    def __init__(self, filename, num, raw_line):
        self.filename = filename
        self.num = num
        assert "\n" not in raw_line, (
            f"Raw line should not have newline: {repr(raw_line)}")
        self.raw_line = raw_line
        without_indent = self.raw_line.lstrip()
        self.indent = self.raw_line[:-len(without_indent)]
        self.start_token, self.end_token = parse_line_tokens(without_indent)
        self._set_text(prefix=f"{self.indent}{self.start_token}")

    def _set_text(self, prefix):
        if self.raw_line.strip() == "":
            self.text = ""
            return
        assert self.raw_line.startswith(prefix), (
            repr(self.raw_line), repr(prefix))
        self.text = self.raw_line[len(prefix):]
        if self.end_token == "*/":
            self.text = self.text[:-len(self.end_token)].rstrip("*").rstrip()

    def reset_indent(self, indent):
        """Readjusts indentation to given string, also adjusting self.text."""
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
        """Shows file and line number with given fixed numbering for line
        number."""
        if num_width == None:
            num_width = len(str(self.num))
        return f"{self.filename}:{self.num + 1:<{num_width}}: {self.raw_line}"


def format_lines(lines, abbrev=False):
    """Prints lines with line numbers.
    If abbrev=True and there are more than 5 lines, this will only show the
    first 2 and last 2 lines with an ellipsis between."""
    num_width = len(str(lines[-1].num))
    text_lines = [x.format(num_width) for x in lines]
    if abbrev and len(lines) > 5:
        text_lines = [
            text_lines[0], text_lines[1], "...",
            text_lines[-2], text_lines[-1]]
    return "\n".join(text_lines)


class UserFormattingError(RuntimeError):
    """Indicates a user formatting error."""
    pass


class MultilineToken:
    """Base class for indicating a multiline token."""
    def __init__(self):
        self.lines = []

    def add_line(self, line):
        """Attempts to add a line. If the line cannot be handled by this token
        type, then False is returned."""
        assert isinstance(line, FileLine), line
        if len(self.lines) > 0:
            if line.num != self.lines[-1].num + 1:
                print(self.lines[-1])
                print(line)
                assert False
        self.lines.append(line)
        return True

    def to_raw_lines(self):
        """Convert to raw lines."""
        return [line.raw_line for line in self.lines]

    def assert_finished(self):
        """Can be used to indicate that the given token has not yet finished
        parsing."""
        pass

    def __str__(self):
        return format_lines(self.lines)

    def __len__(self):
        return len(self.lines)

    def __repr__(self):
        return (
            f"<{self.__class__.__name__} {self.lines[0].filename}:"
            f"{self.lines[0].num + 1}-{self.lines[-1].num + 1}>")


def multiline_token_cls_list():
    """Indicates all non-generic multiline tokens that are relevant to this
    script."""
    # N.B. Does not contain `GenericMultilineToken`.
    return (
        # Docstrings.
        TripleSlashMultilineToken,
        DoubleStarMultilineToken,
        # Comments.
        DoubleSlashMultilineToken,
        SingleStarMultilineToken,
        # Whitespace.
        WhitespaceMultilineToken,
    )


class GenericMultilineToken(MultilineToken):
    """Indicates a set of lines that do not fit any other token type."""
    def add_line(self, line):
        for cls in multiline_token_cls_list():
            if cls().add_line(line):
                return False
        return super().add_line(line)


class WhitespaceMultilineToken(MultilineToken):
    """Indicates a set of lines that are purely whitespace."""
    def add_line(self, line):
        if line.raw_line.strip() == "":
            return super().add_line(line)
        else:
            return False


def dedent_lines(lines, token, require_nonragged):
    """
    Dedents all lines.

    token: For debugging purposes. Will print out an error with debug string.
    require_nonragged: If specified, then the first line must be the
    leftmost indented line.
    """
    text = dedent("\n".join(lines))
    if require_nonragged and len(text) > 0:
        # Ensure that first nonempty line does not start with whitespace
        # (ragged indentation?).
        if text.lstrip("\n")[0] == " ":
            raise UserFormattingError(
                f"Must not have ragged indentation:\n{token}")
    return text


def remove_empty_leading_and_trailing_lines(raw_lines):
    """Removes empty leading and trailing lines."""
    while raw_lines[0].strip() == "":
        del raw_lines[0]
    while raw_lines[-1].strip() == "":
        del raw_lines[-1]


class CommentMultilineToken(MultilineToken):
    """Indicates a comment (non-docstring) multiline token.
    This is an abstract class."""
    pass


class DoubleSlashMultilineToken(CommentMultilineToken):
    """Indicates a block of comments starting with "//"."""
    def add_line(self, line):
        if line.start_token != "//":
            return False
        return super().add_line(line)


class SingleStarMultilineToken(CommentMultilineToken):
    """Indicates a comments of the form "/* ... */"."""
    def __init__(self):
        self._finished = False
        super().__init__()

    def add_line(self, line):
        if len(self.lines) == 0 and (
                line.start_token != "/*" or line.raw_line.endswith("\\")):
            # Don't even try.
            return False
        if self._finished:
            return False
        if line.end_token == "*/":
            self._finished = True
        return super().add_line(line)

    def assert_finished(self):
        assert self._finished, f"Not closed:\n{self}"


class DocstringMultilineToken(MultilineToken):
    """Abstract base class for indicating docstring multiline tokens."""
    def get_docstring_text(self):
        """Gets the docstring text with all preceding "cruft" stripped out.
        Preserves indicated whitespace."""
        text_lines = [line.text for line in self.lines]
        remove_empty_leading_and_trailing_lines(text_lines)
        return dedent_lines(text_lines, self, require_nonragged=True).strip()


class TripleSlashMultilineToken(DocstringMultilineToken):
    """Indicates a block of docstring comments starting with "///"."""
    def add_line(self, line):
        if line.start_token == "///":
            return super().add_line(line)
        else:
            raw = line.raw_line.strip()
            # Not really triple slash, but meh.
            if raw in ("//@{", "//@}"):
                new = f"/// {raw[2:]}"
                line = FileLine(line.filename, line.num, f"{line.indent}{new}")
                return super().add_line(line)
            else:
                return False


class DoubleStarMultilineToken(DocstringMultilineToken):
    """Indicates a multiline docstring comment of the form "/** ... */".
    This may have "*" in intermediate lines."""
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
            remove_empty_leading_and_trailing_lines(text_lines)
            text = text_lines[0].lstrip() + "\n"
            text += dedent_lines(text_lines[1:], self, require_nonragged=False)
            return text.strip()

    def add_line(self, line):
        if len(self.lines) == 0 and line.start_token != "/**":
            return False
        if self._finished:
            return False

        do_add_line = False

        if line.end_token == "*/":
            self._finished = True
            do_add_line = True

        if len(self.lines) == 0:
            do_add_line = True
            if line.text and line.text[0] == "*":
                text = line.text.lstrip("*")
                line = FileLine(
                    line.filename, line.num, f"{line.indent}/**{text}")
        else:
            if self.secondary_type is None:
                if line.start_token == "*":
                    self.secondary_type = "*"
                else:
                    self.secondary_type = ""
            if self.secondary_type == "":
                do_add_line = True
                # Reset indentation to match first line.
                line.reset_indent(self.lines[0].indent)
            else:
                if not do_add_line and line.start_token != "*":
                    lines_str = format_lines(self.lines + [line])
                    raise UserFormattingError(
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


def make_new_multiline_token(line):
    """Creates a new MultilineToken class according to the first class that can
    consume the provided line."""
    for cls in multiline_token_cls_list():
        token = cls()
        if token.add_line(line):
            return token
    token = GenericMultilineToken()
    assert token.add_line(line), line
    return token


def multiline_tokenize(filename, raw_lines):
    """Tokenizes a set of raw lines, which are labeled as part of `filename`,
    into a set of multiline tokens."""
    tokens = []
    active_token = None
    for num, raw_line in enumerate(raw_lines):
        line = FileLine(filename, num, raw_line)
        if active_token is None:
            active_token = make_new_multiline_token(line)
        elif not active_token.add_line(line):
            active_token.assert_finished()
            tokens.append(active_token)
            active_token = make_new_multiline_token(line)
        assert active_token is not None
    active_token.assert_finished()
    tokens.append(active_token)
    return tokens


def reformat_docstring(docstring):
    """Reformats a DocstringMultilineToken into a canonical form."""
    MAX_LEN = 80
    indent = docstring.lines[0].indent
    spacing = ""
    text = docstring.get_docstring_text()
    # Can't have nested comments :(
    # Replace with magical D-style stuff?
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
        new_lines = [f"{indent}/**"] # {first_line}"]
        for line in text_lines[:-1]:
            new_line = f"{indent}{spacing}{line}".rstrip()
            new_lines.append(new_line)
        last_line = text_lines[-1]
        new_lines += maybe_wrap(f"{spacing}{last_line}", " */")
    return new_lines


def reformat_multiline_token(token):
    """Reformats a multiline token for applying / checking lint."""
    if isinstance(token, DocstringMultilineToken):
        # Multi-pass for idempotent.
        # TODO(eric): Fix this.
        tokens = [token]
        first_line = token.lines[0]
        prev_lines = None
        for i in range(3):
            new_lines = reformat_docstring(token)
            if prev_lines is not None:
                if new_lines == prev_lines:
                    break
                else:
                    pass
            token = parse_single_multiline_token(
                new_lines, first_line.filename, first_line.num)
            tokens.append(token)
            prev_lines = new_lines
        else:
            for i, token in enumerate(tokens):
                print(f"i = {i}")
                print(token)
                print("---")
            assert False, "Bug in indempotent check"
        return new_lines
    else:
        return token.to_raw_lines()


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
    tokens = multiline_tokenize("test", block.split("\n"))
    texts = []
    for docstring in tokens:
        if not isinstance(docstring, DocstringMultilineToken):
            continue
        text = "\n".join(reformat_docstring(docstring))
        print(text)
        texts.append(text)
        print("---")
    assert len(texts) == 5, len(texts)
    for text in texts[1:]:
        assert text == texts[0]

    maybe = ["/// Hello /* world */"]
    docstring, = multiline_tokenize("test", maybe)
    new_lines = reformat_multiline_token(docstring)
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
    docstring, = multiline_tokenize("test", yar.split("\n"))
    new_lines = reformat_multiline_token(docstring)
    # Enusre this works...
    text = "\n".join(new_lines)
    print(text)

    ragged = dedent("""\
        /// abc
        /// def
        ///ghe
    """.rstrip())
    token, = multiline_tokenize("test", ragged.split("\n"))
    # Ragged indent.
    try:
        print(token.get_docstring_text())
        assert False
    except UserFormattingError as e:
        assert "ragged indentation" in str(e)
        print(str(e))


class TokenRegex:
    """Simple pattern matcher for a sequence of tokens."""

    class PatternGroup:
        """Base class for a part within given sequence."""
        def try_match(self, xs, index):
            """Takes a sequence and an index in the sequence, and returns the
            number of elements consume, or None if the part did not match."""
            raise NotImplemented

    class Single(PatternGroup):
        """Matches a single token that is indicated by the predicate."""
        def __init__(self, predicate):
            self._predicate = predicate

        def __repr__(self):
            return f"<Single {self._predicate}>"

        def try_match(self, xs, index):
            x = xs[index]
            if self._predicate(x):
                return [x]
            else:
                return None

    class Any(PatternGroup):
        """Matches zero or more continuous tokens that are indicated by the
        predicate."""
        def __init__(self, predicate):
            self._predicate = predicate

        def __repr__(self):
            return f"<Any {self._predicate}>"

        def try_match(self, xs, index):
            group = []
            while self._predicate(xs[index]) and index < len(xs):
                group.append(xs[index])
                index += 1
            return group

    class Match:
        """Indicates a match with a set of pattern groups."""
        def __init__(self):
            self._groups = []

        def add_group(self, group):
            self._groups.append(group)

        def groups(self):
            return list(self._groups)

    def __init__(self, parts):
        self._pattern_groups = list(parts)

    def __repr__(self):
        return f"<TokenRegex {self._pattern_groups}>"

    def find_all(self, xs):
        """Finds all mathches and returns List[Match]."""
        matches = []
        index = 0
        while index < len(xs):
            start_index = index
            match = self.Match()
            for pattern_group in self._pattern_groups:
                if index == len(xs):
                    match = None
                    break
                group = pattern_group.try_match(xs, index)
                if group is not None:
                    index += len(group)
                    match.add_group(group)
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


def reorder_multiline_tokens(tokens, lint_errors):
    """Reorders tokens to a given order (or reports lint errors if linting)."""
    mkdoc_issue = TokenRegex([
        TokenRegex.Single(is_meaningful_docstring_token),
        TokenRegex.Any(is_whitespace_token),
        TokenRegex.Single(is_comment_token_but_not_nolint),
        TokenRegex.Single(is_generic_token_but_not_macro),
    ])
    matches = mkdoc_issue.find_all(tokens)
    for match in matches:
        (doc,), ws, (comment,), (generic,) = match.groups()
        if lint_errors is not None:
            lint_errors.add(
                "ERROR: Docstring must be placed directly next to symbol for "
                "mkdoc.py",
                lines=[doc.lines[-1], comment.lines[0], generic.lines[0]],
            )
        else:
            original = [doc] + ws + [comment, generic]
            start = tokens.index(original[0])
            for x in original:
                tokens.remove(x)
            comment.lines = [
                x for x in comment.lines if x.text.strip() != ""]
            new = [comment] + ws + [doc, generic]
            for i, x in enumerate(new):
                tokens.insert(start + i, x)
    return tokens


def is_whitespace_token(token):
    """Predicate for a whitespace token."""
    return isinstance(token, WhitespaceMultilineToken)


def is_meaningful_docstring_token(token):
    """Predicate for a docstring token that isn't simply a doxygen
    directive."""
    if isinstance(token, DocstringMultilineToken):
        if len(token.lines) == 1:
            if token.lines[0].text.strip().startswith("@"):
                return False
        return True
    return False


def is_generic_token_but_not_macro(token):
    """Predicate for a generic token (possibly code) that is not a
    macro."""
    if isinstance(token, GenericMultilineToken):
        if not token.lines[0].text.startswith("#"):
            return True
    return False


def is_comment_token_but_not_nolint(token):
    """Predicate for a comment token that is not a "nolint" directive.""" 
    if isinstance(token, CommentMultilineToken):
        # TODO(eric.cousineau): Figure out how to make cpplint play nicely with
        # mkdoc.py?
        if token.lines[-1].text.strip().startswith("NOLINTNEXTLINE"):
            return False
        return True
    return False


class LintErrors:
    """Records errors encountered during linting."""
    class Item:
        def __init__(self, text, lines):
            self.text = text
            self.lines = lines

    def __init__(self):
        self.items = []

    def add(self, text, lines):
        self.items.append(self.Item(text, lines))


def print_multiline_tokens(tokens):
    """Prints a list of tokens with newlines in between each."""
    for token in tokens:
        print(token)


def parse_single_multiline_token(raw_lines, filename, start_num):
    """Parse a set of isolated lines into a new token.

    All lines must be consumed in the parsing."""
    token = None
    for i, raw_line in enumerate(raw_lines):
        line = FileLine(filename, start_num + i, raw_line)
        if token is None:
            token = make_new_multiline_token(line)
        else:
            assert token.add_line(line), line
    token.assert_finished()
    return token


def lint_multiline_token(lint_errors, token, new_lines):
    """Detects if there are any changes between `token` and the re-processed
    lines from `new_lines."""
    assert lint_errors is not None
    first_line = token.lines[0]
    # Reparse to ensure that our new token is still valid.
    new_token = parse_single_multiline_token(
        new_lines, first_line.filename, first_line.num)
    # Compare.
    if token.to_raw_lines() != new_token.to_raw_lines():
        lint_errors.add(
            "ERROR: Docstring needs reformatting",
            lines=token.lines[:2],
        )


def check_or_apply_lint(filename, check_lint):
    """Operates on a single file.

    If check_lint is True, will simply print a set of errors if any formatting
    is needed.
    Otherwise, this will mutate the file in place to fix all lint errors.
    """
    with open(filename, "r") as f:
        raw_lines = [x.rstrip() for x in f.readlines()]
    tokens = multiline_tokenize(filename, raw_lines)
    if check_lint:
        lint_errors = LintErrors()
    else:
        lint_errors = None
    tokens = reorder_multiline_tokens(tokens, lint_errors)
    # Replace docstrings with "re-rendered" version.
    new_lines = []
    for token in tokens:
        new_lines_i = reformat_multiline_token(token)
        if lint_errors is not None:
            lint_multiline_token(lint_errors, token, new_lines_i)
        new_lines += new_lines_i
    if check_lint:
        errors = lint_errors.items
        if len(errors) == 0:
            return True
        errors = sorted(errors, key=lambda x: (x.lines[0].num, x.text))
        for i, error in enumerate(errors):
            if i == 3:
                remaining = len(errors) - 3
                print(f"ERROR: There are {remaining} more errors for: "
                      f"{filename}")
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
        source_tree = os.path.join(os.path.dirname(__file__), "../..")
        os.chdir(source_tree)
        result = run(
            ["find", ".", "-name", "*.h"],
            check=True, stdout=PIPE, encoding="utf8")
        filenames = result.stdout.strip().split("\n")
        filenames.sort()
        for filename in list(filenames):
            if filename.startswith(("./attic", "./third_party", "./tools")):
                filenames.remove(filename)

    if filenames == ["<test>"]:
        test()
        return

    bad_filenames = []
    for filename in filenames:
        good = check_or_apply_lint(filename, check_lint=not args.fix)
        if not good:
            bad_filenames.append(filename)
    if bad_filenames:
        if len(bad_filenames) > 5:
            suggested_args = "--all"
        else:
            suggested_args = ' '.join(bad_filenames)
        print(f"To fix errors, please run:")
        print(f"   ./tools/lint/cpp_docstring_lint.py --fix {suggested_args}")
        sys.exit(1)


assert  __name__ == "__main__"
main()
