#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/sorted_pair.h"

namespace pybind11 {
namespace detail {

// Casts `SortedPair<T>` as `Tuple[T]` comprised of `(first, second)`.
template <typename T>
struct type_caster<drake::SortedPair<T>> {
  using Type = drake::SortedPair<T>;
  using InnerCaster = make_caster<T>;

  // N.B. This macro assumes placement in `pybind11::detail`.
  PYBIND11_TYPE_CASTER(Type, _("Tuple[") + type_caster<T>::name + _("]"));

  bool load(handle src, bool convert) {
    if (!convert && !tuple::check_(src)) {
      return false;
    }
    tuple t = reinterpret_borrow<tuple>(src);
    if (t.size() != 2) return false;
    InnerCaster first, second;
    if (!first.load(t[0], convert) || !second.load(t[1], convert)) {
      return false;
    }
    value = Type(first, second);
    return true;
  }

  static handle cast(Type src, return_value_policy policy, handle parent) {
    return make_tuple(InnerCaster::cast(src.first(), policy, parent),
        InnerCaster::cast(src.second(), policy, parent));
  }
};

}  // namespace detail
}  // namespace pybind11
