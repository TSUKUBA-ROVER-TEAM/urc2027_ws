#include "trc2026_manual/launcher_trigger.hpp"

#include <gtest/gtest.h>

#include <limits>

using trc2026_manual::LauncherTrigger;

TEST(LauncherTrigger, StartupHoldAndRepeatedMessagesFireOnlyOnce)
{
  LauncherTrigger trigger;
  trigger.feedback(3.0, 0.0);
  EXPECT_FALSE(trigger.update(true, true, true, 0.0));
  EXPECT_FALSE(trigger.update(true, true, true, 0.1));
  EXPECT_FALSE(trigger.update(true, false, true, 0.2));
  auto target = trigger.update(true, true, true, 0.35);
  ASSERT_TRUE(target);
  EXPECT_DOUBLE_EQ(*target, 0.0);
  for (int i = 0; i < 100; ++i) {
    trigger.feedback(*target, 0.4 + i * 0.05);
    EXPECT_FALSE(trigger.update(true, true, true, 0.4 + i * 0.05));
  }
}

TEST(LauncherTrigger, BusyPressIsDiscardedAndRequiresReleaseAfterCompletion)
{
  LauncherTrigger trigger;
  trigger.feedback(0.0, 0.0);
  trigger.update(true, false, true, 0.0);
  auto target = trigger.update(true, true, true, 0.2);
  ASSERT_TRUE(target);
  trigger.update(true, false, true, 0.3);
  EXPECT_FALSE(trigger.update(true, true, true, 0.45));
  trigger.feedback(*target, 1.2);
  EXPECT_FALSE(trigger.update(true, true, true, 1.2));
  trigger.update(true, false, true, 1.3);
  trigger.feedback(*target, 1.5);
  auto second = trigger.update(true, true, true, 1.5);
  ASSERT_TRUE(second);
  EXPECT_NEAR(*second, *target + 20.0 * 3.14159265358979323846, 1e-10);
}

TEST(LauncherTrigger, ReconnectMissingButtonAndBounceDoNotFire)
{
  LauncherTrigger trigger;
  trigger.feedback(0.0, 0.0);
  trigger.update(true, false, true, 0.0);
  EXPECT_FALSE(trigger.update(true, true, true, 0.05));
  trigger.update(true, false, true, 0.1);
  EXPECT_FALSE(trigger.update(false, false, true, 0.2));
  EXPECT_FALSE(trigger.update(true, true, true, 0.3));
  trigger.update(true, false, true, 0.4);
  trigger.feedback(0.0, 1.0);
  EXPECT_FALSE(trigger.update(true, true, true, 1.0));
}

TEST(LauncherTrigger, MissingStaleNonfiniteFeedbackAndNoControllerConsumePress)
{
  for (int mode = 0; mode < 4; ++mode) {
    LauncherTrigger trigger;
    if (mode == 1) {
      trigger.feedback(0.0, -1.0);
    }
    if (mode == 2) {
      trigger.feedback(std::numeric_limits<double>::quiet_NaN(), 0.0);
    }
    if (mode == 3) {
      trigger.feedback(0.0, 0.0);
    }
    trigger.update(true, false, true, 0.0);
    EXPECT_FALSE(trigger.update(true, true, mode != 3, 0.2));
    trigger.feedback(0.0, 0.3);
    EXPECT_FALSE(trigger.update(true, true, true, 0.3));
  }
}

TEST(LauncherTrigger, CooldownPressIsNotQueued)
{
  LauncherTrigger trigger;
  trigger.feedback(0.0, 0.0);
  trigger.update(true, false, true, 0.0);
  auto target = trigger.update(true, true, true, 0.2);
  ASSERT_TRUE(target);
  trigger.feedback(*target, 0.3);
  trigger.update(true, false, true, 0.3);
  EXPECT_FALSE(trigger.update(true, true, true, 0.5));
  trigger.feedback(*target, 1.2);
  EXPECT_FALSE(trigger.update(true, true, true, 1.2));
}

TEST(LauncherTrigger, FirstPressMovesToZeroBeforeAnyShot)
{
  LauncherTrigger trigger;
  trigger.feedback(17.0, 0.0);
  trigger.update(true, false, true, 0.0);
  const auto zero = trigger.update(true, true, true, 0.2);
  ASSERT_TRUE(zero);
  EXPECT_DOUBLE_EQ(*zero, 0.0);
  // No new command while moving to zero, even after cooldown and a new press.
  trigger.feedback(8.0, 1.0);
  trigger.update(true, false, true, 1.0);
  EXPECT_FALSE(trigger.update(true, true, true, 1.2));
  trigger.feedback(0.0, 1.3);
  EXPECT_FALSE(trigger.update(true, true, true, 1.3));
  trigger.update(true, false, true, 1.4);
  const auto shot = trigger.update(true, true, true, 1.6);
  ASSERT_TRUE(shot);
  EXPECT_NEAR(*shot, 20.0 * 3.14159265358979323846, 1e-10);
}

TEST(LauncherTrigger, RejectedPressDoesNotConsumeInitialZeroMove)
{
  LauncherTrigger trigger;
  trigger.feedback(17.0, 0.0);
  trigger.update(true, false, true, 0.0);
  EXPECT_FALSE(trigger.update(true, true, false, 0.2));
  trigger.update(true, false, true, 0.3);
  trigger.feedback(17.0, 0.5);
  const auto zero = trigger.update(true, true, true, 0.5);
  ASSERT_TRUE(zero);
  EXPECT_DOUBLE_EQ(*zero, 0.0);
}
