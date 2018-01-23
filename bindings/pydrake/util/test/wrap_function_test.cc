#include <functional>
#include <type_traits>

#include <gtest/gtest.h>

#include "drake/bindings/pydrake/util/wrap_function.h"

using namespace std;

namespace drake {

// N.B. Anonymous namespace not used as it makes failure messages
// (static_assert) harder to interpret.

// Will keep argument / return types the same, but homogenize method signatures.
template <typename Func>
auto WrapIdentity(Func&& func) {
  return WrapFunction<wrap_arg_default>(std::forward<Func>(func));
}

#define WTEST(name) GTEST_TEST(WrapFunction, name)

// Function with `void` return type, `int` by value.
void FunctionPtr(int value) {}

WTEST(FunctionPtr) {
  int value = 0;
  WrapIdentity(FunctionPtr)(value);
  EXPECT_EQ(value, 0);
}

// Lambdas / basic functors.
WTEST(Lambda) {
  int value = 0;
  auto func_1_lambda = [](int value) {};
  WrapIdentity(func_1_lambda)(value);
  EXPECT_EQ(value, 0);

  std::function<void (int)> func_1_func = func_1_lambda;
  WrapIdentity(func_1_func)(value);
  EXPECT_EQ(value, 0);
}

// Class methods.
class MyClass {
 public:
  static int MethodStatic(int value) { return value; }
  int MethodMutable(int value) { return value + value_; }
  int MethodConst(int value) const { return value * value_; }

 private:
  int value_{10};
};

WTEST(Methods) {
  int value = 2;

  MyClass c;
  const MyClass& c_const{c};

  // Wrapped signature: Unchanged.
  EXPECT_EQ(WrapIdentity(&MyClass::MethodStatic)(value), 2);
  // Wrapped signature: int (MyClass*, int)
  auto method_mutable = WrapIdentity(&MyClass::MethodMutable);
  EXPECT_EQ(method_mutable(&c, value), 12);
  // method_mutable(&c_const, value);  // Should fail.
  // Wrapped signature: int (const MyClass*, int)
  EXPECT_EQ(WrapIdentity(&MyClass::MethodConst)(&c_const, value), 20);
}

// Move-only arguments.
struct MoveOnlyValue {
  MoveOnlyValue() = default;
  MoveOnlyValue(const MoveOnlyValue&) = delete;
  MoveOnlyValue& operator=(const MoveOnlyValue&) = delete;
  MoveOnlyValue(MoveOnlyValue&&) = default;
  MoveOnlyValue& operator=(MoveOnlyValue&&) = default;
  int value{};
};

void ArgMoveOnly(MoveOnlyValue arg) {
  EXPECT_EQ(arg.value, 1);
}

const int& ArgMoveOnlyConst(const MoveOnlyValue& arg) {
  return arg.value;
}

int& ArgMoveOnlyMutable(MoveOnlyValue& arg) {
  arg.value += 1;
  return arg.value;
}

WTEST(ArgMoveOnly) {
  WrapIdentity(ArgMoveOnly)(MoveOnlyValue{1});
  MoveOnlyValue v{10};
  
  const int& out_const = WrapIdentity(ArgMoveOnlyConst)(v);
  v.value += 10;
  EXPECT_EQ(out_const, 20);

  int& out_mutable = WrapIdentity(ArgMoveOnlyMutable)(v);
  EXPECT_EQ(out_mutable, 21);
  out_mutable += 1;
  EXPECT_EQ(v.value, 22);
}

// Provides a functor which can be default constructed and moved only.
struct MoveOnlyFunctor {
  MoveOnlyFunctor(const MoveOnlyFunctor&) = delete;
  MoveOnlyFunctor& operator=(const MoveOnlyFunctor&) = delete;
  MoveOnlyFunctor(MoveOnlyFunctor&&) = default;
  MoveOnlyFunctor& operator=(MoveOnlyFunctor&&) = default;
  // N.B. Per documentation, cannot overload operator(), as it's ambiguous when
  // attempting to infer arguments.
  void operator()(int& value) const {
    value += 1;
  }
};

WTEST(MoveOnlyFunctor) {
  int value = 0;
  auto wrapped = WrapIdentity(MoveOnlyFunctor{});
  wrapped(value);
  EXPECT_EQ(value, 1);
}

struct ConstFunctor {
  void operator()(int& value) const { value += 1; }
};

WTEST(ConstFunctor) {
  int value = 0;
  const ConstFunctor functor{};
  auto wrapped = WrapIdentity(functor);
  wrapped(value);
  EXPECT_EQ(value, 1);
}


// Test a slightly complicated conversion mechanism.

// Constructible only via brackets (no implicit conversions).
template <typename T>
struct const_ptr {
  static_assert(!std::is_const<T>::value, "Bad (redundant) inference");
  const T* value{};
};

template <typename T>
struct ptr {
  // Ensure that this is not being used in lieu of `ptr` (ensure that
  // our specializiation delegate correctly).
  static_assert(
      !std::is_const<T>::value, "Should be using `const_ptr`");
  T* value{};
};


// Base case: Pass though.
template <typename T, typename = void>
struct wrap_change : public wrap_arg_default<T> {};

template <typename T>
using wrap_change_t =
    detail::wrap_function_impl<wrap_change>::wrap_arg_t<T>;

// Wraps any `const T*` with `const_ptr`, except for `int`.
// SFINAE. Could be achieved with specialization, but using to uphold SFINAE
// contract provided by `WrapFunction`.
template <typename T>
struct wrap_change<const T*, std::enable_if_t<!std::is_same<T, int>::value>> {
  static const_ptr<T> wrap(const T* arg) {
    return {arg};
  }

  static const T* unwrap(const_ptr<T> arg_wrapped) {
    return arg_wrapped.value;
  }
};

// Wraps any `const T&` with `const_ptr`, except for `int`.
// Leverage `const T&` logic, such that we'd get the default template when
// SFINAE prevents matching.
template <typename T>
struct wrap_change<const T&> : public wrap_change<const T*> {
  using Base = wrap_change<const T*>;
  using Wrapped = wrap_change_t<const T*>;

  static Wrapped wrap(const T& arg) { return Base::wrap(&arg); }

  static const T& unwrap(Wrapped arg_wrapped) {
    return *Base::unwrap(arg_wrapped);
  }
};

// Wraps any mutable `T*` with `ptr`.
// N.B. Prevent `const T*` from binding here, since it may be rejected from
// SFINAE.
template <typename T>
struct wrap_change<T*, std::enable_if_t<!std::is_const<T>::value>> {
  static ptr<T> wrap(T* arg) { return {arg}; }
  static T* unwrap(ptr<T> arg_wrapped) { return arg_wrapped.value; }
};

// Wraps any mutable `T&` with `ptr`.
template <typename T>
struct wrap_change<T&> {
  static ptr<T> wrap(T& arg) { return {&arg}; }
  static T& unwrap(ptr<T> arg_wrapped) { return *arg_wrapped.value; }
};

// Test case to exercise `WrapFunction`.
// Mappings:
//   `T*`         -> `ptr<T>` (always)
//   `T&`         -> `ptr<T>` (always).
//   `const T*`   -> `const_ptr<T>`, if `T` is not `int`.
//   `const int*` -> `const int*`
//   `const T&`   -> `const_ptr<T>`, if `T` is not `int`.
//   `const int&` -> `const int*`
template <typename Func>
auto WrapChange(Func&& func) {
  return WrapFunction<wrap_change>(std::forward<Func>(func));
}

template <typename Expected, typename Actual>
void check_type() {
  // Use this function to inspect types when failure is encountered.
  static_assert(std::is_same<Actual, Expected>::value, "Mismatch");
}

GTEST_TEST(WrapFunction, ChangeTypeCheck) {
  // Codify rules above.
  // Use arbitrary T that is not constrained by the rules.
  using T = double;

  check_type<ptr<T>, wrap_change_t<T*>>();
  check_type<ptr<int>, wrap_change_t<int*>>();

  check_type<ptr<T>, wrap_change_t<T&>>();
  check_type<ptr<int>, wrap_change_t<int&>>();

  check_type<const_ptr<T>, wrap_change_t<const T*>>();
  check_type<const int*, wrap_change_t<const int*>>();

  check_type<const_ptr<T>, wrap_change_t<const T&>>();
  check_type<const int*, wrap_change_t<const int&>>();
}

template <typename Return, typename... Args>
struct check_signature {
  template <typename Func>
  static void run(const Func& func) {
    run_impl(detail::infer_function_info(func));
  }

  template <typename IReturn, typename... IArgs, typename Func>
  static void run_impl(
      const detail::function_info<Func, IReturn, IArgs...>& info) {
    check_type<Return, IReturn>();
    using Dummy = int[];
    (void)Dummy{(check_type<Args, IArgs>(), 0)...};
  }
};

class MyClassChange {
 public:
  double* ChangeBasic(
      // double (general case)
      double,
      double*, double&,
      const double*, const double&,
      // int (special case)
      int,
      int*, int&,
      const int*, const int&) {
    return nullptr;
  }
};

WTEST(ChangeBasic) {
  auto wrapped = WrapChange(&MyClassChange::ChangeBasic);
  using check_expected =
      check_signature<
          // Return.
          ptr<double>,
          // self
          ptr<MyClassChange>,
          // double (general case)
          double,
          ptr<double>, ptr<double>,
          const_ptr<double>, const_ptr<double>,
          // int (special case)
          int, ptr<int>, ptr<int>,
          const int*, const int*>;
  check_expected::run(wrapped);
}

}  // namespace drake
