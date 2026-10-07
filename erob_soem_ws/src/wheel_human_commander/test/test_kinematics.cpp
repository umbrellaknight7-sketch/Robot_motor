#include "wheel_human_commander/kinematics.hpp"

#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

using namespace wheel_human_commander;

namespace {
std::string fixture() {
  std::ostringstream s;
  s << "<robot name='fixture'><link name='base_link'/><link name='body'/>"
       "<joint name='torso' type='prismatic'><parent link='base_link'/><child link='body'/>"
       "<origin xyz='0 0 0.1'/><axis xyz='0 0 1'/><limit lower='-0.5' upper='0.5' effort='0' velocity='0'/></joint>";
  const std::vector<std::string> axes = {"0 0 1", "0 1 0", "1 0 0", "0 1 0", "1 0 0", "0 0 1"};
  for (const auto &side : {std::string("Left"), std::string("Right")}) {
    std::string parent = "body";
    for (int i = 0; i < 6; ++i) {
      std::string name =
          side + (i == 0 ? "_Shoulder_Joint"
                         : (i == 5 ? "_Wrist_Joint" : "_Arm_Joint" + std::to_string(i)));
      std::string child = i == 5 ? side + "_Hand_Link" : side + "_Link" + std::to_string(i);
      s << "<link name='" << child << "'/><joint name='" << name
        << "' type='revolute'><parent link='" << parent << "'/><child link='" << child
        << "'/><origin xyz='"
        << (i == 0 ? (side == "Left" ? "0 0.2 0.5" : "0 -0.2 0.5") : "0.1 0 0") << "'/><axis xyz='"
        << axes[i]
        << "'/><limit lower='-3.141592653589793' upper='3.141592653589793' effort='0' velocity='0'/></joint>";
      parent = child;
    }
    s << "<link name='" << side << "_Hand_TCP'/><joint name='" << side
      << "_TCP_Fixed' type='fixed'>"
         "<parent link='"
      << parent << "'/><child link='" << side << "_Hand_TCP'/><origin xyz='0.05 0 0'/></joint>";
  }
  s << "</robot>";
  return s.str();
}

std::vector<KinematicsGroup> groups() {
  std::vector<KinematicsGroup> out;
  for (const auto &side : {std::string("Left"), std::string("Right")}) {
    KinematicsGroup g;
    g.name = side == "Left" ? "left_arm" : "right_arm";
    g.tip = side + "_Hand_TCP";
    g.joints = {side + "_Shoulder_Joint", side + "_Arm_Joint1", side + "_Arm_Joint2",
                side + "_Arm_Joint3",     side + "_Arm_Joint4", side + "_Wrist_Joint"};
    out.push_back(g);
  }
  return out;
}

sensor_msgs::msg::JointState state(const KinematicsModel &model, bool bent = false) {
  sensor_msgs::msg::JointState s;
  s.name = model.joint_names();
  s.position.assign(s.name.size(), 0);
  if (bent) {
    const std::vector<double> values = {.2, -.4, .3, .6, -.2, .1};
    for (const auto &g : groups())
      for (size_t i = 0; i < g.joints.size(); ++i) {
        auto it = std::find(s.name.begin(), s.name.end(), g.joints[i]);
        s.position[it - s.name.begin()] = values[i];
      }
  }
  return s;
}

void set(sensor_msgs::msg::JointState &s, const std::string &name, double value) {
  s.position.at(std::find(s.name.begin(), s.name.end(), name) - s.name.begin()) = value;
}

Eigen::Vector3d translation(const geometry_msgs::msg::PoseStamped &p) {
  return {p.pose.position.x, p.pose.position.y, p.pose.position.z};
}

Eigen::Matrix3d rotation(const geometry_msgs::msg::PoseStamped &p) {
  const auto &q = p.pose.orientation;
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
}
}

TEST(Kinematics, AnalyticFKAndTCP) {
  KinematicsModel m(fixture(), groups());
  auto s = state(m);
  auto tcp = m.forward(s, "Left_Hand_TCP");

  ASSERT_TRUE(tcp) << tcp.message;
  EXPECT_EQ(tcp.pose.header.frame_id, "base_link");
  EXPECT_NEAR(tcp.pose.pose.position.x, .55, 1e-12);
  EXPECT_NEAR(tcp.pose.pose.position.y, .2, 1e-12);
  EXPECT_NEAR(tcp.pose.pose.position.z, .6, 1e-12);

  set(s, "Left_Wrist_Joint", M_PI / 2);
  auto hand = m.forward(s, "Left_Hand_Link");
  tcp = m.forward(s, "Left_Hand_TCP");

  EXPECT_NEAR(tcp.pose.pose.position.x - hand.pose.pose.position.x, 0, 1e-12);
  EXPECT_NEAR(tcp.pose.pose.position.y - hand.pose.pose.position.y, .05, 1e-12);

  auto base = m.forward(s, "base_link");

  ASSERT_TRUE(base);
  EXPECT_NEAR(translation(base.pose).norm(), 0, 1e-12);
  EXPECT_EQ(m.fingerprint(), KinematicsModel(fixture(), groups()).fingerprint());
  EXPECT_EQ(KinematicsModel::pinocchio_version(), "3.8.0");
}

TEST(Kinematics, ConfigurationOrderDoesNotMatter) {
  KinematicsModel m(fixture(), groups());
  auto s = state(m, true);
  auto expected = m.forward(s, "Right_Hand_TCP");
  std::reverse(s.name.begin(), s.name.end());
  std::reverse(s.position.begin(), s.position.end());
  auto actual = m.forward(s, "Right_Hand_TCP");

  ASSERT_TRUE(actual);
  EXPECT_NEAR((translation(actual.pose) - translation(expected.pose)).norm(), 0, 1e-12);
  EXPECT_NEAR((rotation(actual.pose) - rotation(expected.pose)).norm(), 0, 1e-12);
}

TEST(Kinematics, RejectMalformedConfigurations) {
  KinematicsModel m(fixture(), groups());
  auto original = state(m);
  for (int change = 0; change < 7; ++change) {
    auto s = original;
    if (change == 0) {
      s.name.pop_back();
      s.position.pop_back();
    }
    if (change == 1)
      s.name[0] = s.name[1];
    if (change == 2)
      s.name[0] = "unknown";
    if (change == 3)
      s.position[0] = std::numeric_limits<double>::quiet_NaN();
    if (change == 4)
      s.position[0] = std::numeric_limits<double>::infinity();
    if (change == 5)
      s.position[0] = 10;
    if (change == 6)
      s.position.pop_back();
    EXPECT_EQ(m.forward(s, "Left_Hand_TCP").code, KinematicsCode::InvalidInput);
  }

  EXPECT_EQ(m.forward(original, "Left_Wrist_Joint").code, KinematicsCode::InvalidInput);

  original.velocity = {std::numeric_limits<double>::quiet_NaN()};

  EXPECT_TRUE(m.forward(original, "Left_Hand_TCP"));
}

TEST(Kinematics, RejectInvalidModelGroupAndOptions) {
  auto g = groups();
  g[0].joints.push_back("Right_Wrist_Joint");

  EXPECT_THROW(KinematicsModel(fixture(), g), std::invalid_argument);

  g = groups();
  g[0].joints.push_back(g[0].joints[0]);

  EXPECT_THROW(KinematicsModel(fixture(), g), std::invalid_argument);

  g = groups();
  g[0].tip = "unknown";

  EXPECT_THROW(KinematicsModel(fixture(), g), std::invalid_argument);

  g = groups();
  g.push_back(g[0]);

  EXPECT_THROW(KinematicsModel(fixture(), g), std::invalid_argument);

  auto xml = fixture();
  auto pos = xml.find("base_link");
  xml.replace(pos, 9, "wrongroot");

  EXPECT_THROW(KinematicsModel(xml, groups()), std::invalid_argument);

  IkOptions o;
  o.max_iterations = 0;

  EXPECT_THROW(KinematicsModel(fixture(), groups(), o), std::invalid_argument);

  o = IkOptions{};
  o.damping = std::numeric_limits<double>::quiet_NaN();

  EXPECT_THROW(KinematicsModel(fixture(), groups(), o), std::invalid_argument);
}

TEST(Kinematics, JacobianMatchesIndependentFiniteDifferences) {
  KinematicsModel m(fixture(), groups());
  auto s = state(m, true);
  const double eps = 1e-6;
  for (const auto &g : groups()) {
    auto J = m.jacobian(s, g.tip);

    ASSERT_TRUE(J);

    for (size_t i = 0; i < J.joint_names.size(); ++i) {
      auto plus = s, minus = s;
      auto k = std::find(s.name.begin(), s.name.end(), J.joint_names[i]) - s.name.begin();
      plus.position[k] += eps;
      minus.position[k] -= eps;
      auto a = m.forward(plus, g.tip), b = m.forward(minus, g.tip);

      ASSERT_TRUE(a);
      ASSERT_TRUE(b);

      Eigen::Vector3d dp = (translation(a.pose) - translation(b.pose)) / (2 * eps);
      Eigen::AngleAxisd angle(rotation(a.pose) * rotation(b.pose).transpose());
      Eigen::Vector3d dr = angle.axis() * angle.angle() / (2 * eps);
      for (int row = 0; row < 3; ++row) {
        EXPECT_NEAR(J.rows[row][i], dp[row], 1e-7);
        EXPECT_NEAR(J.rows[row + 3][i], dr[row], 1e-7);
      }
    }
  }
}

TEST(Kinematics, BothArmsBothModesAndHeldJoints) {
  KinematicsModel m(fixture(), groups());
  auto goal = state(m, true);
  for (const auto &g : groups())
    for (auto mode : {IkMode::PositionOnly, IkMode::FullPose}) {
      auto target = m.forward(goal, g.tip);

      ASSERT_TRUE(target);

      auto seed = goal;
      for (const auto &name : g.joints) {
        auto i = std::find(seed.name.begin(), seed.name.end(), name) - seed.name.begin();
        seed.position[i] += .04;
      }
      auto result = m.inverse(seed, g.name, target.pose, mode);

      ASSERT_TRUE(result) << result.message;

      auto check = m.forward(result.solution, g.tip);

      ASSERT_TRUE(check);
      EXPECT_LE((translation(check.pose) - translation(target.pose)).norm(), 1e-4);

      if (mode == IkMode::FullPose)
        EXPECT_LE(
            Eigen::AngleAxisd(rotation(check.pose).transpose() * rotation(target.pose)).angle(),
            .001);
      for (size_t i = 0; i < seed.name.size(); ++i)
        if (std::find(g.joints.begin(), g.joints.end(), seed.name[i]) == g.joints.end()) {
          auto j =
              std::find(result.solution.name.begin(), result.solution.name.end(), seed.name[i]) -
              result.solution.name.begin();
          EXPECT_DOUBLE_EQ(result.solution.position[j], seed.position[i]);
        }
    }
}

TEST(Kinematics, TargetValidationAndQuaternionNormalization) {
  KinematicsModel m(fixture(), groups());
  auto s = state(m, true);
  auto target = m.forward(s, "Left_Hand_TCP").pose;
  auto bad = target;
  bad.header.frame_id = "world";

  EXPECT_EQ(m.inverse(s, "left_arm", bad, IkMode::FullPose).code, KinematicsCode::InvalidInput);

  bad = target;
  bad.pose.orientation.x = bad.pose.orientation.y = bad.pose.orientation.z =
      bad.pose.orientation.w = 0;
  EXPECT_EQ(m.inverse(s, "left_arm", bad, IkMode::FullPose).code, KinematicsCode::InvalidInput);

  bad.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();

  EXPECT_TRUE(m.inverse(s, "left_arm", bad, IkMode::PositionOnly));

  bad = target;
  bad.pose.position.x = std::numeric_limits<double>::infinity();

  EXPECT_EQ(m.inverse(s, "left_arm", bad, IkMode::PositionOnly).code, KinematicsCode::InvalidInput);

  auto &q = target.pose.orientation;
  q.x *= 3;
  q.y *= 3;
  q.z *= 3;
  q.w *= 3;

  EXPECT_TRUE(m.inverse(s, "left_arm", target, IkMode::FullPose));
  EXPECT_EQ(m.inverse(s, "unknown", target, IkMode::FullPose).code, KinematicsCode::InvalidInput);
  EXPECT_EQ(m.inverse(s, "left_arm", target, static_cast<IkMode>(99)).code,
            KinematicsCode::InvalidInput);
}

TEST(Kinematics, UnreachableAndSingularTargetsHaveNoSolution) {
  IkOptions options;
  options.timeout_seconds = 1;
  options.max_iterations = 30;
  KinematicsModel m(fixture(), groups(), options);
  auto s = state(m);
  auto target = m.forward(s, "Left_Hand_TCP").pose;
  target.pose.position.x = 100;
  auto r = m.inverse(s, "left_arm", target, IkMode::FullPose);

  EXPECT_EQ(r.code, KinematicsCode::NoConvergence);
  EXPECT_TRUE(r.solution.name.empty());
  EXPECT_TRUE(r.solution.position.empty());

  target = m.forward(s, "Left_Hand_TCP").pose;

  EXPECT_TRUE(m.inverse(s, "left_arm", target, IkMode::FullPose));
}

TEST(Kinematics, JointBoundsAreRespected) {
  KinematicsModel m(fixture(), groups());
  auto s = state(m, true);
  set(s, "Left_Wrist_Joint", M_PI);
  auto target = m.forward(s, "Left_Hand_TCP").pose;
  auto seed = s;
  set(seed, "Left_Arm_Joint3", .61);
  auto r = m.inverse(seed, "left_arm", target, IkMode::FullPose);

  ASSERT_TRUE(r) << r.message;

  for (size_t i = 0; i < r.solution.name.size(); ++i) {
    if (r.solution.name[i] != "torso") {
      EXPECT_LE(r.solution.position[i], M_PI);
      EXPECT_GE(r.solution.position[i], -M_PI);
    }
  }
}

TEST(Kinematics, IterationAndTimeBudgetsAreExplicit) {
  IkOptions o;
  o.max_iterations = 1;
  o.timeout_seconds = 1;
  KinematicsModel m(fixture(), groups(), o);
  auto s = state(m, true);
  auto target = m.forward(s, "Left_Hand_TCP").pose;
  target.pose.position.x += .3;
  auto r = m.inverse(s, "left_arm", target, IkMode::PositionOnly);

  EXPECT_EQ(r.code, KinematicsCode::NoConvergence);
  EXPECT_TRUE(r.solution.name.empty());
  EXPECT_LE(r.iterations, 1);

  o.timeout_seconds = 1e-12;
  KinematicsModel timed(fixture(), groups(), o);
  r = timed.inverse(s, "left_arm", target, IkMode::PositionOnly);

  EXPECT_EQ(r.code, KinematicsCode::Timeout);
  EXPECT_TRUE(r.solution.name.empty());
}
