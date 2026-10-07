#include "wheel_human_commander/ownership_manager.hpp"
#include "wheel_human_commander/goal_validator.hpp"
#include "wheel_human_commander/motion.hpp"

#include <gtest/gtest.h>
#include <limits>

using namespace wheel_human_commander;

TEST(Ownership, LeaseRenewalAndRevocation) {
  OwnershipManager o;
  auto a = o.acquire("vla", 10);

  ASSERT_FALSE(a.empty());
  EXPECT_TRUE(o.valid(a, 11));
  EXPECT_FALSE(o.valid(a, 12));
  EXPECT_FALSE(o.renew(a, 12));
  EXPECT_TRUE(o.expired(12));

  o.revoke();
  auto b = o.acquire("teleop", 12);

  EXPECT_NE(a, b);
  EXPECT_FALSE(o.valid(a, 12));
  EXPECT_TRUE(o.renew(b, 13));
  EXPECT_TRUE(o.valid(b, 14.9));

  o.revoke();

  EXPECT_FALSE(o.valid(b, 14));
}

TEST(Ownership, NoImplicitTakeover) {
  OwnershipManager o;
  auto a = o.acquire("a", 0);

  EXPECT_TRUE(o.acquire("b", .1).empty());
  EXPECT_EQ(o.owner(), "a");
  EXPECT_FALSE(o.renew("wrong", .2));
  EXPECT_TRUE(o.valid(a, 1));
}

TEST(State, CompleteFreshFiniteFeedback) {
  RobotStateCache s({"a", "b"});
  sensor_msgs::msg::JointState m;
  m.name = {"b", "a"};
  m.position = {2, 1};
  m.velocity = {0, 0};
  s.update(m, 1);

  ASSERT_TRUE(s.fresh(1.1, .5));
  EXPECT_EQ(s.positions()[0], 1);
  EXPECT_FALSE(s.fresh(2, .5));

  m.name = {"a", "a"};
  s.update(m, 2);

  EXPECT_FALSE(s.fresh(2, .5));
}

TEST(Goals, SubsetsAndInvalidRequests) {
  erob::BusConfig c;
  c.drives.resize(2);
  for (size_t i = 0; i < 2; ++i) {
    auto &d = c.drives[i];
    d.joint = i ? "b" : "a";
    d.min_position = -2;
    d.max_position = 2;
    d.max_velocity = d.max_velocity_offset = 1;
    d.max_torque_offset = 1;
    d.hold_acceleration = 2;
    d.hold_jerk = 20;
  }
  RobotStateCache s({"a", "b"});
  sensor_msgs::msg::JointState m;
  m.name = {"a", "b"};
  m.position = {0, .3};
  m.velocity = {0, 0};
  s.update(m, 0);
  wheel_human_interfaces::action::MoveJoints::Goal g;
  g.joint_names = {"a"};
  g.positions = {.1};
  g.duration.sec = 2;
  auto r = validate_goal(g, c, s);

  ASSERT_TRUE(r);
  EXPECT_EQ(r.position[1], .3);

  g.torque_joint_names = {"b"};
  g.torque_offsets = {.2};

  EXPECT_FALSE(validate_goal(g, c, s));

  g.torque_joint_names = {"a"};

  EXPECT_TRUE(validate_goal(g, c, s));

  g.positions[0] = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(validate_goal(g, c, s));

  g.gravity_compensation = true;

  EXPECT_EQ(validate_goal(g, c, s).code, Code::NotImplemented);
}

TEST(Motion, PlaceholdersNeverPretendToSucceed) {
  EXPECT_EQ(gravity_compensation({}).code, Code::NotImplemented);
  EXPECT_EQ(generate_advanced_trajectory({}, {}).code, Code::NotImplemented);
}

TEST(State, PartialFeedbackDoesNotRefreshOrInvalidateOtherJoints) {
  RobotStateCache s({"body", "wrist"});
  sensor_msgs::msg::JointState m;
  m.name = {"body"};
  m.position = {.1};
  m.velocity = {0};
  s.update(m, 1);

  EXPECT_TRUE(s.fresh("body", 1.1, .5));
  EXPECT_FALSE(s.fresh("wrist", 1.1, .5));

  m.name = {"wrist"};
  s.update(m, 1.4);

  EXPECT_TRUE(s.fresh(1.4, .5));

  s.update(m, 1.6);

  EXPECT_FALSE(s.fresh("body", 1.6, .5));
  EXPECT_TRUE(s.fresh("wrist", 1.6, .5));

  m.name = {"unrelated"};
  m.position.clear();
  s.update(m, 1.7);

  EXPECT_TRUE(s.fresh("wrist", 1.7, .5));

  s.invalidate({"wrist"});

  EXPECT_FALSE(s.fresh("wrist", 1.7, .5));
}
