#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
#
#  Syntax: mkdoc.py [-I<path> ..] [-quiet] [.. a list of header files ..]
#
#  Extract documentation from C++ header files to use it in Python bindings
#

from collections import OrderedDict, defaultdict
import os
import sys
import platform
import re
from tempfile import NamedTemporaryFile
import textwrap

from clang import cindex
from clang.cindex import AccessSpecifier, CursorKind

CLASS_KINDS = [
    CursorKind.CLASS_DECL,
    CursorKind.STRUCT_DECL,
    CursorKind.CLASS_TEMPLATE,
]

FUNCTION_KINDS = [
    CursorKind.FUNCTION_DECL,
    CursorKind.FUNCTION_TEMPLATE,
    CursorKind.CONVERSION_FUNCTION,
    CursorKind.CXX_METHOD,
    CursorKind.CONSTRUCTOR,
]


RECURSE_LIST = [
    CursorKind.TRANSLATION_UNIT,
    CursorKind.NAMESPACE,
    CursorKind.CLASS_DECL,
    CursorKind.STRUCT_DECL,
    CursorKind.ENUM_DECL,
    CursorKind.CLASS_TEMPLATE
]

PRINT_LIST = CLASS_KINDS + FUNCTION_KINDS + [
    CursorKind.ENUM_DECL,
    CursorKind.ENUM_CONSTANT_DECL,
    CursorKind.FIELD_DECL
]

CPP_OPERATORS = {
    '<=': 'le', '>=': 'ge', '==': 'eq', '!=': 'ne', '[]': 'array',
    '+=': 'iadd', '-=': 'isub', '*=': 'imul', '/=': 'idiv', '%=':
    'imod', '&=': 'iand', '|=': 'ior', '^=': 'ixor', '<<=': 'ilshift',
    '>>=': 'irshift', '++': 'inc', '--': 'dec', '<<': 'lshift', '>>':
    'rshift', '&&': 'land', '||': 'lor', '!': 'lnot', '~': 'bnot',
    '&': 'band', '|': 'bor', '+': 'add', '-': 'sub', '*': 'mul', '/':
    'div', '%': 'mod', '<': 'lt', '>': 'gt', '=': 'assign', '()': 'call'
}

CPP_OPERATORS = OrderedDict(
    sorted(CPP_OPERATORS.items(), key=lambda t: -len(t[0])))


def d(s):
    return s.decode('utf8')


class Symbol(object):
    def __init__(self, node, name, include, line, comment):
        self.node = node
        self.name = name
        self.include = include
        self.line = line
        self.comment = comment

    def sorting_key(self):
        return (self.name, self.include, self.line)

SKIP_FULL_NAMES = [
    'Eigen',
    'detail',
    'internal',
    'std',
]

SKIP_PARTIAL_NAMES = [
    'operator new',
    'operator delete',
    'operator=',
    'operator->',
]

SKIP_ACCESS = [
    AccessSpecifier.PRIVATE,
]


def is_accepted_symbol(node):
    name = d(node.spelling)
    if name in SKIP_FULL_NAMES:
        return False
    for bad in SKIP_PARTIAL_NAMES:
        if bad in name:
            return False
    if node.access_specifier in SKIP_ACCESS:
        return False
    # TODO(eric.cousineau): Remove `node.is_default_method()`? May make things
    # unstable.
    # TODO(eric.cousineau): Figure out how to strip forward declarations.
    return True


def eprint(*args):
    print(*args, file=sys.stderr)


def sanitize_name(name):
    name = re.sub(r'type-parameter-0-([0-9]+)', r'T\1', name)
    for k, v in CPP_OPERATORS.items():
        name = name.replace('operator%s' % k, 'operator_%s' % v)
    name = re.sub('<.*>', '', name)
    name = name.replace('::', '_')
    name = ''.join([ch if ch.isalnum() else '_' for ch in name])
    name = re.sub('_+', '_', name)
    return name


def process_comment(comment):
    result = ''

    # Remove C++ comment syntax
    leading_spaces = float('inf')
    for s in comment.expandtabs(tabsize=4).splitlines():
        s = s.strip()
        if s.startswith('/*'):
            s = s[2:].lstrip('*')
        elif s.endswith('*/'):
            s = s[:-2].rstrip('*')
        elif s.startswith('///'):
            s = s[3:]
        if s.startswith('*'):
            s = s[1:]
        if len(s) > 0:
            leading_spaces = min(leading_spaces, len(s) - len(s.lstrip()))
        result += s + '\n'

    if leading_spaces != float('inf'):
        result2 = ""
        for s in result.splitlines():
            result2 += s[leading_spaces:] + '\n'
        result = result2

    # Doxygen tags
    cpp_group = '([\w:]+)'
    param_group = '([\[\w:\]]+)'

    s = result
    s = re.sub(r'[@\\]c\s+%s' % cpp_group, r'``\1``', s)
    s = re.sub(r'[@\\]p\s+%s' % cpp_group, r'``\1``', s)
    s = re.sub(r'[@\\]a\s+%s' % cpp_group, r'*\1*', s)
    s = re.sub(r'[@\\]e\s+%s' % cpp_group, r'*\1*', s)
    s = re.sub(r'[@\\]em\s+%s' % cpp_group, r'*\1*', s)
    s = re.sub(r'[@\\]b\s+%s' % cpp_group, r'**\1**', s)
    s = re.sub(r'[@\\]ingroup\s+%s' % cpp_group, r'', s)
    s = re.sub(r'[@\\]param%s?\s+%s' % (param_group, cpp_group),
               r'\n\n$Parameter ``\2``:\n\n', s)
    s = re.sub(r'[@\\]tparam%s?\s+%s' % (param_group, cpp_group),
               r'\n\n$Template parameter ``\2``:\n\n', s)
    s = re.sub(r'[@\\]retval\s+%s' % cpp_group,
               r'\n\n$Returns ``\1``:\n\n', s)

    for in_, out_ in {
        'result': 'Returns',
        'returns': 'Returns',
        'return': 'Returns',
        'authors': 'Authors',
        'author': 'Authors',
        'copyright': 'Copyright',
        'date': 'Date',
        'note': 'Note',
        'remarks': 'Remark',
        'remark': 'Remark',
        'sa': 'See also',
        'see': 'See also',
        'extends': 'Extends',
        'throws': 'Throws',
        'throw': 'Throws'
    }.items():
        s = re.sub(r'[@\\]%s\s*' % in_, r'\n\n$%s:\n\n' % out_, s)

    s = re.sub(r'[@\\]details\s*', r'\n\n', s)
    s = re.sub(r'[@\\]brief\s*', r'', s)
    s = re.sub(r'[@\\]short\s*', r'', s)
    s = re.sub(r'[@\\]ref\s*', r'', s)

    s = re.sub(r'[@\\]code\s?(.*?)\s?[@\\]endcode',
               r"```\n\1\n```\n", s, flags=re.DOTALL)

    s = re.sub(r'%(\S+)', r'\1', s)

    # HTML/TeX tags
    s = re.sub(r'<tt>(.*?)</tt>', r'``\1``', s, flags=re.DOTALL)
    s = re.sub(r'<pre>(.*?)</pre>', r"```\n\1\n```\n", s, flags=re.DOTALL)
    s = re.sub(r'<em>(.*?)</em>', r'*\1*', s, flags=re.DOTALL)
    s = re.sub(r'<b>(.*?)</b>', r'**\1**', s, flags=re.DOTALL)
    s = re.sub(r'[@\\]f\$(.*?)[@\\]f\$', r'$\1$', s, flags=re.DOTALL)
    s = re.sub(r'<li>', r'\n\n* ', s)
    s = re.sub(r'</?ul>', r'', s)
    s = re.sub(r'</li>', r'\n\n', s)

    s = s.replace('``true``', '``True``')
    s = s.replace('``false``', '``False``')

    # Exceptions
    s = s.replace('std::bad_alloc', 'MemoryError')
    s = s.replace('std::domain_error', 'ValueError')
    s = s.replace('std::exception', 'RuntimeError')
    s = s.replace('std::invalid_argument', 'ValueError')
    s = s.replace('std::length_error', 'ValueError')
    s = s.replace('std::out_of_range', 'ValueError')
    s = s.replace('std::range_error', 'ValueError')

    # Re-flow text
    wrapper = textwrap.TextWrapper()
    wrapper.expand_tabs = True
    wrapper.replace_whitespace = True
    wrapper.drop_whitespace = True
    wrapper.width = 70
    wrapper.initial_indent = wrapper.subsequent_indent = ''

    result = ''
    in_code_segment = False
    for x in re.split(r'(```)', s):
        if x == '```':
            if not in_code_segment:
                result += '```\n'
            else:
                result += '\n```\n\n'
            in_code_segment = not in_code_segment
        elif in_code_segment:
            result += x.strip()
        else:
            for y in re.split(r'(?: *\n *){2,}', x):
                wrapped = wrapper.fill(re.sub(r'\s+', ' ', y).strip())
                if len(wrapped) > 0 and wrapped[0] == '$':
                    result += wrapped[1:] + '\n'
                    wrapper.initial_indent = \
                        wrapper.subsequent_indent = ' ' * 4
                else:
                    if len(wrapped) > 0:
                        result += wrapped + '\n\n'
                    wrapper.initial_indent = wrapper.subsequent_indent = ''
    return result.rstrip().lstrip('\n')


def get_full_name(node):
    name = [d(node.spelling)]
    p = node.semantic_parent
    while p.kind != CursorKind.TRANSLATION_UNIT:
        name.insert(0, d(p.spelling))
        p = p.semantic_parent
    return tuple(name)


class SymbolTree(object):

    def __init__(self):
        self.root = SymbolTree.Leaf()

    def append(self, symbol):
        leaf = self.root
        for piece in symbol.name:
            leaf = leaf.get_child(piece)
        leaf.symbols.append(symbol)

    class Leaf(object):
        def __init__(self):
            self.symbols = []
            self.children_map = defaultdict(SymbolTree.Leaf)

        def get_child(self, piece):
            return self.children_map[piece]


def print_symbols(name, leaf, level=0):
    if len(leaf.symbols) == 0:
        full_name = name
    else:
        top = leaf.symbols[0]
        full_pieces = top.name
        assert name == full_pieces[-1]
        full_name = "::".join(full_pieces)
        # Override variable.
        if top.node.kind == CursorKind.CONSTRUCTOR:
            name = "ctor"

    name = sanitize_name(name)

    indent = '  '*level
    iprint = lambda s: print((indent + s).rstrip())
    iprint('// {}'.format(full_name))
    modifier = ""
    if level == 0:
        modifier = "constexpr "
    iprint('{}struct /* {} */ {{'.format(name))
    iprint('')
    for i, symbol in enumerate(leaf.symbols):
        assert full_pieces == symbol.name
        var = "doc";
        if i > 0:
            var += "_{}".format(i + 1)
        delim = "\n"
        if "\n" not in symbol.comment and len(symbol.comment) < 40:
            delim = " "
        iprint('  // {}:{}'.format(symbol.include, symbol.line))
        iprint('  const char* {} ={}R"doc({})doc";'.format(var, delim, symbol.comment))
        iprint('')
    keys = sorted(leaf.children_map.keys())
    for key in keys:
        child = leaf.children_map[key]
        print_symbols(key, child, level=level+1)
    iprint('}} {};'.format(name))
    iprint('')


def extract(include_map, node, output):
    if node.kind == CursorKind.TRANSLATION_UNIT:
        for i in node.get_children():
            extract(include_map, i, output)
        return
    assert node.location.file is not None
    filename = d(node.location.file.name)
    include = include_map.get(filename)
    if include is None:
        return
    if not is_accepted_symbol(node):
        return
    if node.kind in RECURSE_LIST:
        for i in node.get_children():
            extract(include_map, i, output)
    if node.kind in PRINT_LIST:
        if len(node.spelling) > 0:
            comment = d(node.raw_comment) if node.raw_comment is not None else ''
            comment = process_comment(comment)
            name = get_full_name(node)
            line = node.location.line
            output.append(Symbol(node, name, include, line, comment))


def drake_genfile_path_to_include_path(filename):
    # TODO(eric.cousineau): Is there a simple way to generalize this, given
    # include paths?
    pieces = filename.split('/')
    assert pieces.count('drake') == 1
    drake_index = pieces.index('drake')
    return '/'.join(pieces[drake_index:])


class FileDict(object):
    def __init__(self, items):
        self._d = {self._key(file): value for file, value in items}

    def _key(self, file):
        return os.path.realpath(os.path.abspath(file))

    def get(self, file, default=None):
        return self._d.get(self._key(file), default)

    def __contains__(self, file):
        return self._key(file) in self._d

    def __getitem__(self, file):
        key = self._key(file)
        return self._d[key]


if __name__ == '__main__':
    parameters = ['-x', 'c++', '-D__MKDOC_PY__']
    filenames = []

    if platform.system() == 'Darwin':
        dev_path = '/Applications/Xcode.app/Contents/Developer/'
        lib_dir = dev_path + 'Toolchains/XcodeDefault.xctoolchain/usr/lib/'
        sdk_dir = dev_path + 'Platforms/MacOSX.platform/Developer/SDKs'
        libclang = lib_dir + 'libclang.dylib'

        if os.path.exists(libclang):
            cindex.Config.set_library_path(os.path.dirname(libclang))

        if os.path.exists(sdk_dir):
            sysroot_dir = os.path.join(sdk_dir, next(os.walk(sdk_dir))[1][0])
            parameters.append('-isysroot')
            parameters.append(sysroot_dir)

    quiet = False
    std = '-std=c++11'

    for item in sys.argv[1:]:
        if item == '-quiet':
            quiet = True
        elif item.startswith('-std='):
            std = item
        elif item.startswith('-'):
            parameters.append(item)
        else:
            filenames.append(item)

    parameters.append(std)

    if len(filenames) == 0:
        print('Syntax: %s [.. a list of header files ..]' % sys.argv[0],
              file=sys.stderr)
        exit(-1)

    print('''#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py from pybind11.

#define __EXPAND(x)                                      x
#define __COUNT(_1, _2, _3, _4, _5, _6, _7, COUNT, ...)  COUNT
#define __VA_SIZE(...)                                   __EXPAND(__COUNT(__VA_ARGS__, 7, 6, 5, 4, 3, 2, 1))
#define __CAT1(a, b)                                     a ## b
#define __CAT2(a, b)                                     __CAT1(a, b)
#define __DOC1(n1)                                       __doc_##n1
#define __DOC2(n1, n2)                                   __doc_##n1##_##n2
#define __DOC3(n1, n2, n3)                               __doc_##n1##_##n2##_##n3
#define __DOC4(n1, n2, n3, n4)                           __doc_##n1##_##n2##_##n3##_##n4
#define __DOC5(n1, n2, n3, n4, n5)                       __doc_##n1##_##n2##_##n3##_##n4##_##n5
#define __DOC6(n1, n2, n3, n4, n5, n6)                   __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6
#define __DOC7(n1, n2, n3, n4, n5, n6, n7)               __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6##_##n7
#define DOC(...)                                         __EXPAND(__EXPAND(__CAT2(__DOC, __VA_SIZE(__VA_ARGS__)))(__VA_ARGS__))

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
''')

    if False:
        import pickle
        with open('/tmp/data.pkl', 'wb') as f:
            data = dict(
                parameters=parameters,
                filenames=filenames,
            )
            pickle.dump(data, f)

        from os import execvp
        os.environ['BAZEL_RUNFILES'] = os.getcwd()
        os.environ['PATH'] = '/home/eacousineau/bin:/home/eacousineau/.local/bin:'
        sys.path.insert(0, os.path.dirname(__file__))
        os.environ['PYTHONPATH'] = ':'.join(sys.path)
        execvp('jupyter', ['jupyter', 'notebook', '/home/eacousineau/proj/tri/repo/branches/drake/review/drake/bindings/pydrake/cindex_stuff.ipynb'])
        exit(1)

    includes = list(map(drake_genfile_path_to_include_path, filenames))
    include_map = FileDict(zip(filenames, includes))
    # TODO(eric.cousineau): Sort files based on include path?
    with NamedTemporaryFile('w') as include_file:
        for include in includes[:]:
            if True: #include in ["drake/systems/primitives/sine.h"]:
                include_file.write("#include \"{}\"\n".format(include))
        include_file.flush()
        if not quiet:
            print("Parse header...", file=sys.stderr)
        index = cindex.Index(
            cindex.conf.lib.clang_createIndex(False, True))
        tu = index.parse(include_file.name, parameters)

    if not quiet:
        print("Extract relevant symbols...", file=sys.stderr)
    output = SymbolTree()
    extract(include_map, tu.cursor, output)
    print_symbols('root', output.root)

    print('''
#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
''')
