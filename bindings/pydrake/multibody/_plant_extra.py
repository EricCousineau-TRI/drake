import textwrap as _textwrap
import typing as _typing

import pydrake.autodiffutils as _ad
import pydrake.symbolic as _sym
from pydrake.common import cpp_template as _cpp_template
from pydrake.common.value import Value as _Value
from pydrake.common.deprecation import deprecated as _deprecated

_PARAM_LIST = (
    (float,),
    (_ad.AutoDiffXd,),
    (_sym.Expression,),
)

_deprecation_msg = """
VectorExternallyAppliedSpatialForced and
Value[VectorExternallyAppliedSpatialForced] are deprecated. Please use
list() and Value[List[ExternallyAppliedSpatialForce]] instead.
The deprecated code will be removed from Drake on or after 2020-09-01.
""".strip()


@_cpp_template.TemplateClass.define(
    "VectorExternallyAppliedSpatialForced_", _PARAM_LIST)
def VectorExternallyAppliedSpatialForced_(param):
    T, = param
    preferred_cls = _typing.List[ExternallyAppliedSpatialForce_[T]]

    class Impl(preferred_cls):
        _PREFERRED_CLS = _typing.List[ExternallyAppliedSpatialForce_[T]]
        def __init__(self, value=None):
            if value is not None:
                super().__init__()
            else:
                super().__init__(value)

    Impl.__doc__ = f"Warning:\n{_textwrap.indent(_deprecation_msg, '    ')}"
    return Impl


def _deprecate_vector_instantiations():
    for param in _PARAM_LIST:
        # Deprecate old type.
        cls, _ = VectorExternallyAppliedSpatialForced_.deprecate_instantiation(
            param, _deprecation_msg)
        # Add deprecated alias to Value[].
        _Value.add_instantiation(param, cls._PREFERRED_CLS)
        _Value.deprecate_instantiation(param, _deprecation_msg)


_deprecate_vector_instantiations()
