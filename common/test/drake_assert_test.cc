#include "drake/common/drake_assert.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

GTEST_TEST(DrakeAssertTest, MatchingConfigTest) {
#ifdef DRAKE_ASSERT_IS_ARMED
  EXPECT_TRUE(::drake::kDrakeAssertIsArmed);
  EXPECT_FALSE(::drake::kDrakeAssertIsDisarmed);
#else
  EXPECT_FALSE(::drake::kDrakeAssertIsArmed);
  EXPECT_TRUE(::drake::kDrakeAssertIsDisarmed);
#endif
#ifdef DRAKE_ASSERT_IS_DISARMED
  EXPECT_FALSE(::drake::kDrakeAssertIsArmed);
  EXPECT_TRUE(::drake::kDrakeAssertIsDisarmed);
#else
  EXPECT_TRUE(::drake::kDrakeAssertIsArmed);
  EXPECT_FALSE(::drake::kDrakeAssertIsDisarmed);
#endif
}

// Note that Drake's styleguide forbids death tests, but our only choice here
// is to use death tests because our implementation is documented to abort().

GTEST_TEST(DrakeAssertDeathTest, DemandTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_DEATH(
      { DRAKE_DEMAND(false); },
      "abort: Failure at .*drake_assert_test.cc:.. in TestBody..: "
      "condition 'false' failed");
}

struct NotBoolConvertible {};
struct BoolConvertible { operator bool() const { return true; } };

using drake::internal::bool_convertible_v;

GTEST_TEST(DrakeAssertDeathTest, BoolConvertible) {
  static_assert(bool_convertible_v<BoolConvertible>, "");
  static_assert(!bool_convertible_v<NotBoolConvertible>, "");
  static_assert(bool_convertible_v<std::unique_ptr<int>>, "");
  static_assert(bool_convertible_v<std::function<void()>>, "");
}

GTEST_TEST(DrakeAssertDeathTest, AssertSyntaxTest) {
  // These should compile.
  DRAKE_ASSERT((2 + 2) == 4);
  DRAKE_ASSERT(BoolConvertible());
  // Test additional complex types.
  auto nonempty_ptr = std::make_unique<int>(10);
  DRAKE_ASSERT(nonempty_ptr);
  std::function<void()> nonempty_function = []() {};
  DRAKE_ASSERT(nonempty_function);
}

GTEST_TEST(DrakeAssertDeathTest, AssertFalseTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  if (::drake::kDrakeAssertIsArmed) {
    ASSERT_DEATH(
        { DRAKE_ASSERT(2 + 2 == 5); },
        "abort: Failure at .*drake_assert_test.cc:.. in TestBody..: "
        "condition '2 \\+ 2 == 5' failed");
  } else {
    DRAKE_ASSERT(2 + 2 == 5);
  }
}

GTEST_TEST(DrakeAssertDeathTest, AssertVoidTestArmed) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  if (::drake::kDrakeAssertIsArmed) {
    ASSERT_DEATH(
        { DRAKE_ASSERT_VOID(::abort()); },
        "");
  } else {
    DRAKE_ASSERT_VOID(::abort());
  }
}

}  // namespace
