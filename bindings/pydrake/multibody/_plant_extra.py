import typing as _typing

import pydrake.autodiffutils as _ad
import pydrake.symbolic as _sym
from pydrake.common import cpp_template as _cpp_template

_PARAM_LIST = (
    (float,),
    (_ad.AutoDiffXd,),
    (_sym.Expression,),
)


@_cpp_template.TemplateClass.define(
    "VectorExternallyAppliedSpatialForced_", _PARAM_LIST)
def VectorExternallyAppliedSpatialForced_(param):
    T, = param

    class Impl(_typing.List[ExternallyAppliedSPatialForced_[T]]):
        @deprecated
        def __init__(self, value=None):
            if value is not None:
                super().__init__()
            else:
                super().__init__(value)

    # TODO(eric.cousineau): Register type with Value[]?
    return Impl
