"""Provides Sphinx plugins / monkey patches to:
 - Remove excessive bases when documenting inheritance
 - Document parameterized bindings of templated methods / classes

For guidance, see:
 - http://www.sphinx-doc.org/en/master/extdev/appapi.html#sphinx.application.Sphinx.add_autodocumenter
"""

# TODO(eric.cousineau): How to document only protected methods?
# e.g. `LeafSystem` only consists of private things to overload, but it's
# important to be user-visible.

from __future__ import print_function

from collections import namedtuple
import re

from sphinx.locale import _
import sphinx.domains.python as pydoc
from sphinx.ext import autodoc

from pydrake.util.cpp_template import TemplateBase, TemplateMethod


def rindex(s, sub):
    """Reverse index of a substring."""
    return len(s) - s[::-1].index(sub) - len(sub)


def patch(obj, name):
    """Decorator to patch a method of a class."""

    def decorator(f):
        original = getattr(obj, name)

        def method(*args, **kwargs):
            return f(original, *args, **kwargs)

        setattr(obj, name, method)
        return method

    return decorator


def spoof_instancemethod(module, name, f):
    """Spoofs an instancemethod, generally to just set documentation attributes
    """

    def tmp(*args, **kwargs):
        return f(*args, **kwargs)

    tmp.__module__ = module
    tmp.__name__ = name
    tmp.__doc__ = f.__doc__
    return tmp


class IrregularExpression(object):
    """Provides analogous parsing to `autodoc.py_ext_sig_re` and
    `pydoc.py_sig_re`, but permits nested parsing for class-like directives to
    work with the munged names.

    These are meant to be used to monkey-patch existing compiled regular
    expressions.
    """

    FakeMatch = namedtuple('FakeMatch', 'groups')

    py_sig_old = autodoc.py_ext_sig_re
    py_sig = re.compile(
        r'''^     (\w.*?) \s*               # symbol
                  (?:
                      \((.*)\)              # optional: arguments
                      (?:\s* -> \s* (.*))?  # return annotation
                  )? $
              ''', re.VERBOSE)

    def __init__(self, extended):
        """
        Args:
            extended: For use in `autodoc` (returns explicit reST module name
                scope).
        """
        self.extended = extended

    def match(self, full):
        """Tests if a string matches `full`. If not, returns None."""
        # TODO(eric.cousineau): See if there's a way to speed this up with
        # pybind or re.Scanner (to tokenize).
        m = self.py_sig.match(full)
        if not m:
            return None
        s, arg, retann = m.groups()
        # Extract module name using a greedy match.
        explicit_modname = None
        if "::" in s:
            pos = rindex(s, "::") + 2
            explicit_modname = s[:pos]
            s = s[pos:]
        # Extract {path...}.{base}, accounting for brackets.
        path = ''
        base = ''
        num_open = 0
        i = 0
        for c in s:
            if num_open == 0 and c.isspace() or c == '(':
                break
            if num_open == 0 and c == '.':
                path += base + "."
                base = ''
            else:
                if c == '[':
                    num_open += 1
                elif c == ']':
                    num_open -= 1
                base += c
            i += 1
        if not base:
            # Nothing worth keeping.
            return None
        if not path:
            # Clear out.
            path = None
        if self.extended:
            groups = (explicit_modname, path, base, arg, retann)
        else:
            assert explicit_modname is None
            groups = (path, base, arg, retann)
        return self.FakeMatch(lambda: groups)


class TemplateDocumenter(autodoc.ModuleLevelDocumenter):
    """Specializes `Documenter` for templates from `cpp_template`.
    """
    objtype = 'template'
    member_order = autodoc.ClassDocumenter.member_order
    directivetype = 'template'

    # Take priority over attributes.
    priority = 1 + autodoc.AttributeDocumenter.priority

    option_spec = {
        'show-all-instantiations': autodoc.bool_option,
    }
    # Permit propagation of class-specific propreties.
    option_spec.update(autodoc.ClassDocumenter.option_spec)

    @classmethod
    def can_document_member(cls, member, membername, isattr, parent):
        """Overrides base to check for template objects."""
        return isinstance(member, TemplateBase)

    def get_object_members(self, want_all):
        """Overrides base to return instantiations from templates."""
        members = []
        is_method = isinstance(self.object, TemplateMethod)
        for param in self.object.param_list:
            instantiation = self.object[param]
            if is_method:
                # Hack naming of methods.
                instantiation = spoof_instancemethod(
                    self.object._module_name,
                    self.object._instantiation_name(param),
                    instantiation)
            members.append((instantiation.__name__, instantiation))
            if not self.options.show_all_instantiations:
                break
        return False, members

    def check_module(self):
        """Overrides base to always show template objects."""
        # TODO(eric.cousineau): Filter out `TemplateBase` instances based on
        # their originating module. `autodoc` won't catch it normally because
        # an instance does not have `__module__` normally defined.
        return True

    def add_directive_header(self, sig):
        """Overrides base to add a line to indicate instantiations."""
        autodoc.ModuleLevelDocumenter.add_directive_header(self, sig)
        sourcename = self.get_sourcename()
        self.add_line(u'', sourcename)
        names = []
        for param in self.object.param_list:
            # TODO(eric.cousineau): Use attribute aliasing already present in
            # autodoc.
            rst = ":class:`{}`".format(self.object._instantiation_name(param))
            names.append(rst)
        self.add_line(
            u"   Instantiations: {}".format(", ".join(names)), sourcename)


def tpl_attrgetter(obj, name, *defargs):
    """Attribute getter hook for autodoc to permit accessing instantiations via
    instantiation names.

    In ideal world, we'd be able to override instance names easily; however,
    since Sphinx aims to permit either sweeping automation (`automodule`) or
    specific instances (`autoclass`), we have to try and get it to play nice
    with string retrieval.

    Note:
        We cannot call `.. autoclass:: obj.MyTemplate[param]`, because this
    getter is constrained to `TemplateBase` instances.
    """
    # N.B. Rather than try to evaluate parameters from the string, we instead
    # match based on instantiation name.
    # Because methods in Python2 cannot have their `__name__` overwritten, we
    # use hackery with `spoof_instancemethod`.
    if "[" in name:
        assert name.endswith(']'), name
        for param in obj.param_list:
            inst = obj[param]
            if obj._instantiation_name(param) == name:
                return inst
        assert False, (
            "Not a template?",
            param, obj.param_list,
            inst.__name__, name)
    return autodoc.safe_getattr(obj, name, *defargs)


@patch(autodoc.ClassDocumenter, 'add_directive_header')
def patch_add_directive_header(original, self, sig):
    """Patches display of bases for classes to strip out pybind11 meta classes
    from bases.
    """
    if self.doc_as_attr:
        self.directivetype = 'attribute'
    autodoc.Documenter.add_directive_header(self, sig)
    # add inheritance info, if wanted
    if not self.doc_as_attr and self.options.show_inheritance:
        sourcename = self.get_sourcename()
        self.add_line(u'', sourcename)
        bases = getattr(self.object, '__bases__', None)
        if not bases:
            return
        bases = [b.__module__ in ('__builtin__', 'builtins') and
                 u':class:`%s`' % b.__name__ or
                 u':class:`%s.%s`' % (b.__module__, b.__name__)
                 for b in bases
                 if b.__name__ != "pybind11_object"]
        if not bases:
            return
        self.add_line(_(u'   Bases: %s') % ', '.join(bases),
                      sourcename)


def setup(app):
    # Register directive so we can pretty-print template declarations.
    pydoc.PythonDomain.directives['template'] = pydoc.PyClasslike
    # Register custom attribute retriever.
    app.add_autodoc_attrgetter(TemplateBase, tpl_attrgetter)
    app.add_autodocumenter(TemplateDocumenter)
    # Hack regular expressions to make them irregular (nested).
    autodoc.py_ext_sig_re = IrregularExpression(extended=True)
    pydoc.py_sig_re = IrregularExpression(extended=False)
    return dict(parallel_read_safe=True)
