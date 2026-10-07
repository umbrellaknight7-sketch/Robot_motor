#include "erob_controllers/trajectory_sampler.hpp"

#include <gtest/gtest.h>

using namespace erob_controllers;

namespace {
std::array<Limits, capacity> limits() {
  std::array<Limits, capacity> l{};
  l[0] = {-2, 2, 1, 2, 20, 1, .2, .01, .05};
  return l;
}

TEST(Sampler, BoundsAndSynchronizedOffsets) {
  TrajectorySampler s;
  s.configure(1, limits());
  Values q{}, v{};
  s.activate(q, v, 0);
  Request r;
  r.sequence = 1;
  r.duration = 2;
  r.position[0] = .2;
  r.torque[0] = -.1;

  EXPECT_TRUE(s.feasible(r, q, v));

  auto fast = r;
  fast.duration = .01;

  EXPECT_FALSE(s.feasible(fast, q, v));

  s.tick(0, q, v, &r, true, .01, 3);
  for (int i = 1; i <= 2200; ++i) {
    q = s.state().position;
    v = s.state().velocity_offset;
    s.tick(i * .001, q, v, nullptr, true, .01, 3);
  }

  EXPECT_EQ(s.state().outcome, Outcome::Succeeded);
  EXPECT_NEAR(s.state().position[0], .2, 1e-10);
  EXPECT_NEAR(s.state().torque[0], -.1, 1e-10);
  EXPECT_EQ(s.state().velocity_offset[0], 0);
}

TEST(Sampler, WatchdogAndBoundedStop) {
  TrajectorySampler s;
  s.configure(1, limits());
  Values q{}, v{};
  s.activate(q, v, 0);
  Request r;
  r.sequence = 1;
  r.duration = 2;
  r.position[0] = .5;
  r.torque[0] = .2;
  s.tick(0, q, v, &r, true, .01, 3);
  for (int i = 1; i <= 2500; ++i) {
    q = s.state().position;
    v = s.state().velocity_offset;
    s.tick(i * .001, q, v, nullptr, i < 700, .01, 3);
  }

  EXPECT_EQ(s.state().phase, Phase::Holding);
  EXPECT_EQ(s.state().outcome, Outcome::Aborted);
  EXPECT_TRUE(s.state().neutral);
  EXPECT_LT(s.state().position[0], .5);
}

TEST(Sampler, StopCannotBeMistakenForOldCompletion) {
  TrajectorySampler s;
  s.configure(1, limits());
  Values q{}, v{};
  s.activate(q, v, 0);
  Request r;
  r.stop = true;
  r.sequence = 7;
  s.tick(0, q, v, &r, true, .01, 3);

  EXPECT_EQ(s.state().generation, 7u);
  EXPECT_EQ(s.state().phase, Phase::Stopping);

  for (int i = 1; i < 500; ++i)
    s.tick(i * .001, q, v, nullptr, true, .01, 3);
  EXPECT_EQ(s.state().phase, Phase::Holding);
  EXPECT_TRUE(s.state().neutral);
}
}

TEST(Sampler, DerivativeMatchesPositionAndTorqueRamp) {
  TrajectorySampler s;
  s.configure(1, limits());
  Values q{}, v{};
  s.activate(q, v, 0);
  Request r;
  r.sequence = 1;
  r.duration = 2;
  r.position[0] = .2;
  r.torque[0] = -.3;
  s.tick(0, q, v, &r, true, .01, 3);
  for (int i = 1; i < 2000; ++i) {
    double old = s.state().position[0];
    double rate = s.state().velocity_offset[0];
    q = s.state().position;
    v = s.state().velocity_offset;
    s.tick(i * .001, q, v, nullptr, true, .01, 3);

    EXPECT_NEAR((s.state().position[0] - old) / .001, (rate + s.state().velocity_offset[0]) / 2,
                1e-6);
    EXPECT_NEAR(s.state().torque[0], -1.5 * s.state().position[0], 1e-9);
  }
}

TEST(Sampler, RetainedTorqueAlsoRequiresHeartbeat) {
  TrajectorySampler s;
  s.configure(1, limits());
  Values q{}, v{};
  s.activate(q, v, 0);
  Request r;
  r.sequence = 1;
  r.duration = .5;
  r.torque[0] = .3;
  s.tick(0, q, v, &r, true, .01, 3);
  for (int i = 1; i <= 800; ++i)
    s.tick(i * .001, q, v, nullptr, true, .01, 3);
  ASSERT_EQ(s.state().phase, Phase::Holding);
  ASSERT_FALSE(s.state().neutral);

  s.tick(.801, q, v, nullptr, false, .01, 3);

  EXPECT_EQ(s.state().phase, Phase::Stopping);
  EXPECT_EQ(s.state().outcome, Outcome::None);

  for (int i = 802; i <= 1300; ++i)
    s.tick(i * .001, q, v, nullptr, false, .01, 3);
  EXPECT_EQ(s.state().phase, Phase::Holding);
  EXPECT_TRUE(s.state().neutral);
}

TEST(Sampler, CannotAcknowledgeUnconfirmedStop) {
  TrajectorySampler s;
  s.configure(1, limits());
  Values q{}, v{};
  s.activate(q, v, 0);
  v[0] = .5;
  Request r;
  r.stop = true;
  r.sequence = 1;
  s.tick(0, q, v, &r, true, .01, .1);
  for (int i = 1; i < 1500; ++i)
    s.tick(i * .001, q, v, nullptr, true, .01, .1);
  EXPECT_EQ(s.state().phase, Phase::Fault);
  EXPECT_EQ(s.state().outcome, Outcome::Aborted);
}
