#pragma once

#include <functional>
#include <type_traits>
#include <utility>

#include "drake/bindings/pydrake/util/function_inference.h"

namespace drake {
namespace pydrake {
namespace detail {

// Implementation for wrapping a function by scanning and replacing arguments
// based on their types.
template <
    template <typename...> class wrap_arg_policy, bool use_functions = true>
struct wrap_function_impl {
  // By default `wrap_arg_functions` is the same as `wrap_arg_policy`. However,
  // below we specialize it for the case when `T` is of the form
  // `std::function<F>`.
  // N.B. This must precede `wrap_type`.
  template <typename T>
  struct wrap_arg_functions : public wrap_arg_policy<T> {};

  template <typename T>
  using wrap_arg = std::conditional_t<
      use_functions, wrap_arg_functions<T>, wrap_arg_policy<T>>;

  // Provides wrapped argument type.
  // Uses `Extra` to specialize within class scope to intercept `void`.
  template <typename T, typename Extra>
  struct wrap_type {
    using type = decltype(wrap_arg<T>::wrap(std::declval<T>()));
  };

  // Intercept `void`, since `declval<void>()` is invalid.
  template <typename Extra>
  struct wrap_type<void, Extra> {
    using type = void;
  };

  // Convenience helper type.
  template <typename T>
  using wrap_type_t = typename wrap_type<T, void>::type;

  // Determines which overload should be used, since we cannot wrap a `void`
  // type using `wrap_arg<void>::wrap()`.
  template <typename Return>
  static constexpr bool enable_wrap_output =
      !std::is_same<Return, void>::value;

  // Specialization for callbacks of the form `std::function<>`.
  // @note We could generalize this using SFINAE for functors of any form, but
  // that complicates the details for a relatively low ROI.
  template <typename Return, typename ... Args>
  struct wrap_arg_functions<const std::function<Return(Args...)>&> {
    // Define types explicit, since `auto` is not easily usable as a return type
    // (compilers struggle with inference).
    using Func = std::function<Return(Args...)>;
    using WrappedFunc =
        std::function<wrap_type_t<Return> (wrap_type_t<Args>...)>;

    static WrappedFunc wrap(const Func& func) {
      return wrap_function_impl::run(infer_function_info(func));
    }

    // Unwraps a `WrappedFunc`, also unwrapping the return value.
    // @note We use `Defer` so that we can use SFINAE without a disptach method.
    template <typename Defer = Return>
    static Func unwrap(
        const WrappedFunc& func_wrapped,
        std::enable_if_t<enable_wrap_output<Defer>, void*> = {}) {
      return [func_wrapped](Args... args) -> Return {
        return wrap_arg_functions<Return>::unwrap(
            func_wrapped(wrap_arg_functions<Args>::wrap(
                std::forward<Args>(args))...));
      };
    }

    // Specialization / overload of above, but not wrapping the return value.
    template <typename Defer = Return>
    static Func unwrap(
        const WrappedFunc& func_wrapped,
        std::enable_if_t<!enable_wrap_output<Defer>, void*> = {}) {
      return [func_wrapped](Args... args) {
        func_wrapped(wrap_arg_functions<Args>::wrap(
            std::forward<Args>(args))...);
      };
    }
  };

  // Ensure that we also wrap `std::function<>` returned by value.
  template <typename Signature>
  struct wrap_arg_functions<std::function<Signature>>
      : public wrap_arg_functions<const std::function<Signature>&> {};

  // Wraps function arguments and the return value.
  // Generally used when `Return` is non-void.
  template <typename Func, typename Return, typename ... Args>
  static auto run(function_inference::info<Func, Return, Args...>&& info,
      std::enable_if_t<enable_wrap_output<Return>, void*> = {}) {
    // N.B. Since we do not use the `mutable` keyword with this lambda,
    // any functors passed in *must* provide `operator()(...) const`.
    auto func_wrapped =
        [func_f = std::forward<Func>(info.func)]
        (wrap_type_t<Args>... args_wrapped) -> wrap_type_t<Return> {
      return wrap_arg<Return>::wrap(
          func_f(wrap_arg<Args>::unwrap(
                  std::forward<wrap_type_t<Args>>(args_wrapped))...));
    };
    return func_wrapped;
  }

  // Wraps function arguments, but not the return value.
  // Generally used when `Return` is void.
  template <typename Func, typename Return, typename ... Args>
  static auto run(function_inference::info<Func, Return, Args...>&& info,
      std::enable_if_t<!enable_wrap_output<Return>, void*> = {}) {
    auto func_wrapped =
        [func_f = std::forward<Func>(info.func)]
        (wrap_type_t<Args>... args_wrapped) -> Return {
      return func_f(wrap_arg<Args>::unwrap(
              std::forward<wrap_type_t<Args>>(args_wrapped))...);
    };
    return func_wrapped;
  }
};

}  // namespace detail

/// Wraps the types used in a function signature to produce a new function with
/// wrapped arguments and return value (if non-void). The wrapping is based on
/// `wrap_arg_policy`.
/// Any types that are of the form `std::function<F>` will be recursively
/// wrapped, such that callbacks will be of a wrapped form (arguments and
/// return types wrapped). The original form of the callbacks will still be
/// called in the wrapped callback.
/// @tparam wrap_arg_policy
///   User-supplied argument wrapper, that must supply the static functions
///   `wrap(Arg arg) -> Wrapped` and `unwrap(Wrapped wrapped) -> Arg`.
///   `Arg arg` is the original argument, and `Wrapped wrapped` is the wrapped
///   / transformed argument type.
///   N.B. This template template parameter uses a parameter pack to allow
///   for SFINAE. If passing a `using` template alias, ensure that the alias
///   template template parameter uses a parameter pack of the *exact* same
///   form.
/// @tparam use_functions
///   If true (default), will recursively wrap callbacks. If your policy
///   provides handling for functions, then you should set this to false.
/// @param func
///   Functor to be wrapped. Returns a function with wrapped arguments and
///   return type. If functor is a method pointer, it will return a function of
///   the form `Return ([const] Class* self, ...)`.
/// @return Wrapped function lambda.
///   N.B. Construct a `std::function<>` from this if you encounter inference
///   issues downstream of this method.
template <
    template <typename...> class wrap_arg_policy, bool use_functions = true,
    typename Func = void>
auto WrapFunction(Func&& func) {
  // TODO(eric.cousineau): Create an overload with `type_pack<Args...>` to
  // handle overloads, to disambiguate when necessary.
  return detail::wrap_function_impl<wrap_arg_policy, use_functions>::run(
      detail::infer_function_info(std::forward<Func>(func)));
}

/// Default case for argument wrapping, with pure pass-through. Consider
/// inheriting from this for base cases.
/// N.B. `Wrapped` is not necessary, but is used for demonstration purposes.
template <typename T>
struct wrap_arg_default {
  using Wrapped = T;
  static Wrapped wrap(T arg) { return std::forward<T&&>(arg); }
  static T unwrap(Wrapped arg_wrapped) {
    return std::forward<Wrapped&&>(arg_wrapped);
  }
  // N.B. `T` rather than `T&&` is used as arguments here as it behaves well
  // with primitive types, such as `int`.
};

/// Policy for explicitly wrapping functions for a given policy.
template <template <typename...> class wrap_arg_policy, typename Signature>
using wrap_arg_function =
    typename detail::wrap_function_impl<wrap_arg_policy>::
        template wrap_arg<std::function<Signature>>;

}  // namespace pydrake
}  // namespace drake
