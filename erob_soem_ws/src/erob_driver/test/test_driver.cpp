#include "erob_driver/config.hpp"
#include "erob_driver/driver.hpp"
#include "erob_driver/pdo.hpp"
#include "erob_driver/topology.hpp"

#include <algorithm>
#include <chrono>
#include <gtest/gtest.h>
#include <limits>
#include <thread>

using namespace erob;

namespace {
BusConfig config() {
  return load_config(EROB_TEST_CONFIG);
}

struct Rig {
  BusConfig c = config();
  MockTransport t;
  Engine e{c, t};
  Nanoseconds now = ms(1);
  uint64_t gen = 0;

  Rig() {
    e.configure();
    e.tick(now);
  }

  void step(const std::vector<Command> *cmd = nullptr) {
    now += c.cycle_ns;
    CommandFrame f;
    if (cmd) {
      f = {*cmd, now, ++gen};
    }
    e.tick(now, cmd ? &f : nullptr);
  }

  void start() {
    ASSERT_TRUE(e.activate(now));

    for (int i = 0; i < 510; ++i)
      step();
    ASSERT_EQ(e.state().health.phase, Phase::Active);
  }

  std::vector<Command> commands(Mode mode = Mode::CSP, double target = 0) {
    std::vector<Command> v(15);
    for (auto &x : v) {
      x.mode = mode;
      x.claimed = true;
      if (mode == Mode::CSP)
        x.position = target;
      if (mode == Mode::CSV)
        x.velocity = target;
      if (mode == Mode::CST)
        x.effort = target;
    }
    return v;
  }
};

TEST(Config, Topology15) {
  auto c = config();

  ASSERT_EQ(c.drives.size(), 15);
  EXPECT_EQ(c.drives[6].branch, "A");
  EXPECT_EQ(c.drives[7].branch, "B");
  EXPECT_EQ(c.drives[11].logical_id, 12);
  EXPECT_EQ(c.drives[12].branch, "C");
  EXPECT_EQ(c.drives[14].joint, "motor_0f_joint");
}

TEST(Config, RejectInvalidConfiguration) {
  auto c = config();
  c.drives[1].logical_id = 1;

  EXPECT_THROW(validate_config(c), std::runtime_error);

  c = config();
  c.drives[0].position_counts_per_rad = 0;

  EXPECT_THROW(validate_config(c), std::runtime_error);

  c = config();
  c.cycle_ns = 700000;

  EXPECT_THROW(validate_config(c), std::runtime_error);
}

TEST(Config, HardwareActivationGates) {
  auto c = config();
  c.backend = "soem";
  c.interface = "fake0";

  EXPECT_THROW(validate_activation(c), std::runtime_error);

  c.topology_verified = c.stop_verified = c.watchdog_verified = c.padding_verified = true;
  c.hub_model = "fixture";
  c.esi_reference = "fixture.xml";

  EXPECT_THROW(validate_activation(c), std::runtime_error);

  for (auto &d : c.drives)
    d.verified = true;
  EXPECT_NO_THROW(validate_activation(c));
}

TEST(Discovery, OrderAndInfrastructureIndependent) {
  auto c = config();
  MockTransport t;
  auto dev = t.discover(c);
  dev.push_back({50, 0, {1, 2, 3, 0}, {0}, "extra infrastructure", "", 0, false, false});
  std::reverse(dev.begin(), dev.end());
  auto m = match_devices(c, dev);

  EXPECT_EQ(m.front(), 2);
  EXPECT_EQ(m.back(), 16);
}

TEST(Discovery, MissingMotorOrBranch) {
  auto c = config();
  MockTransport t;
  auto dev = t.discover(c);
  dev.pop_back();

  EXPECT_THROW(match_devices(c, dev), std::runtime_error);

  dev = t.devices;
  dev.erase(dev.begin() + 1, dev.begin() + 8);

  EXPECT_THROW(match_devices(c, dev), std::runtime_error);
}

TEST(Discovery, SwappedIdentityAndAmbiguousPath) {
  auto c = config();
  MockTransport t;
  auto dev = t.discover(c);
  std::swap(dev[1].path, dev[8].path);

  EXPECT_THROW(match_devices(c, dev), std::runtime_error);

  dev = t.devices;
  dev[2].path = dev[1].path;

  EXPECT_THROW(match_devices(c, dev), std::runtime_error);

  dev = t.devices;
  dev[2].index = dev[1].index;

  EXPECT_THROW(match_devices(c, dev), std::runtime_error);
}

TEST(Discovery, ZeroSerialIsNotAnIdentity) {
  auto c = config();
  MockTransport t;
  for (auto &d : c.drives) {
    d.require_serial = false;
    d.identity.serial = 0;
  }
  auto dev = t.discover(c);

  EXPECT_NO_THROW(match_devices(c, dev));

  dev[2].path = dev[1].path;

  EXPECT_THROW(match_devices(c, dev), std::runtime_error);
}

TEST(Discovery, UnsupportedModeOrIdentity) {
  auto c = config();
  MockTransport t;
  auto d = t.discover(c);
  d[1].supported_modes = 0x80;

  EXPECT_THROW(match_devices(c, d), std::runtime_error);

  d = t.devices;
  ++d[3].identity.product;

  EXPECT_THROW(match_devices(c, d), std::runtime_error);
}

TEST(Pdo, ExplicitSignedLittleEndian) {
  RxPdo p{0x1234, -12345678, INT32_MIN, INT16_MIN, Mode::CST};
  auto b = encode(p);

  EXPECT_EQ(rx_wire_size(false), 14u);
  EXPECT_EQ(b.size(), 20u);
  EXPECT_EQ(b[0], 0x34);
  EXPECT_EQ(b[1], 0x12);
  EXPECT_EQ(b[12], 10);
  EXPECT_EQ(b[13], 0);

  auto q = decode(b);

  EXPECT_EQ(q.position, p.position);
  EXPECT_EQ(q.velocity, p.velocity);
  EXPECT_EQ(q.torque, p.torque);

  TxPdo v{0x27, 0xa000, INT32_MAX, -77, -123, Mode::CSV};
  auto t = encode(v);

  EXPECT_EQ(t.size(), 16);
  EXPECT_EQ(t[15], 0);

  auto w = decode(t);

  EXPECT_EQ(w.error, 0xa000);
  EXPECT_EQ(w.velocity, -77);
  EXPECT_EQ(w.torque, -123);
}

TEST(Pdo, MappingAndSizeFailures) {
  auto c = config();
  MockTransport t;
  t.refuse_mapping = true;
  Engine e(c, t);

  EXPECT_THROW(e.configure(), std::runtime_error);

  t.refuse_mapping = false;
  t.wrong_size = true;

  EXPECT_THROW(e.configure(), std::runtime_error);
}

TEST(Conversion, DirectionOffsetRoundTrip) {
  auto d = config().drives[0];
  d.direction = -1;
  d.zero_counts = 1000;
  Command c{Mode::CSP, 0.7, 0, 0, true};
  RxPdo out;

  ASSERT_TRUE(command_valid(c, d));

  apply_command(c, d, out);

  EXPECT_NEAR(joint_position(out.position, d), c.position, 1 / d.position_counts_per_rad);

  c.mode = Mode::CSV;
  c.velocity = 0.4;
  apply_command(c, d, out);

  EXPECT_NEAR(joint_velocity(out.velocity, d), 0.4, 1 / d.velocity_counts_per_rad_s);

  c.mode = Mode::CST;
  c.effort = 0.25;
  apply_command(c, d, out);

  EXPECT_NEAR(joint_effort(out.torque, d), 0.25, 1e-9);
}

TEST(Conversion, LimitsOverflowAndUnknownEffort) {
  auto d = config().drives[0];
  Command c{Mode::CSP, 4, 0, 0, true};

  EXPECT_FALSE(command_valid(c, d));

  c.position = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(command_valid(c, d));

  c.position = 1;
  d.position_counts_per_rad = 1e20;

  EXPECT_FALSE(command_valid(c, d));

  c = {Mode::CST, 0, 0, 0.2, true};
  d.effort_verified = false;

  EXPECT_FALSE(command_valid(c, d));
}

TEST(Lifecycle, EnableDelayAndNoAutomaticMotion) {
  Rig r;

  ASSERT_TRUE(r.e.activate(r.now));

  for (int i = 0; i < 499; ++i)
    r.step();
  EXPECT_EQ(r.e.state().health.phase, Phase::Enabling);

  for (int i = 0; i < 20; ++i)
    r.step();
  EXPECT_EQ(r.e.state().health.phase, Phase::Active);

  for (const auto &v : r.t.values)
    EXPECT_EQ(v.position, 0);
}

TEST(Lifecycle, DisableDelayAndStateConfirmation) {
  Rig r;
  r.start();
  r.e.deactivate(r.now);
  for (int i = 0; i < 300; ++i)
    r.step();
  EXPECT_EQ(r.e.state().health.phase, Phase::Disabling);

  for (int i = 0; i < 10; ++i)
    r.step();
  EXPECT_EQ(r.e.state().health.phase, Phase::Inactive);

  for (auto v : r.t.values)
    EXPECT_FALSE(enabled(v.status));
}

TEST(Lifecycle, EnableTimeout) {
  Rig r;
  r.t.freeze_enable = true;

  ASSERT_TRUE(r.e.activate(r.now));

  for (int i = 0; i < 10010; ++i)
    r.step();
  EXPECT_EQ(r.e.state().health.fault, Fault::TransitionTimeout);
}

TEST(Control, PositionAll15) {
  Rig r;
  r.start();
  auto cmd = r.commands(Mode::CSP, 0.1);
  for (int i = 0; i < 5; ++i)
    r.step(&cmd);
  for (auto s : r.e.state().joints)
    EXPECT_NEAR(s.position, 0.1, 2e-5);
}

TEST(Control, VelocityAndEffortAll15) {
  Rig r;
  r.start();
  auto cmd = r.commands(Mode::CSV, 0.2);
  for (int i = 0; i < 20; ++i)
    r.step(&cmd);
  for (auto s : r.e.state().joints)
    EXPECT_NEAR(s.velocity, 0.2, 2e-5);
  cmd = r.commands(Mode::CSV, 0);
  for (int i = 0; i < 120; ++i)
    r.step(&cmd);
  cmd = r.commands(Mode::CST, 0.3);
  for (int i = 0; i < 10; ++i)
    r.step(&cmd);
  for (auto s : r.e.state().joints)
    EXPECT_NEAR(s.effort, 0.3, 1e-9);
}

TEST(Control, ModeAcknowledgmentBeforeTargets) {
  Rig r;
  r.start();
  r.t.mode_delay_cycles = 10;
  auto cmd = r.commands(Mode::CSV, 0.4);
  for (int i = 0; i < 8; ++i) {
    r.step(&cmd);

    EXPECT_EQ(r.t.last_outputs[0].velocity, 0);
  }
  for (int i = 0; i < 10; ++i)
    r.step(&cmd);
  EXPECT_EQ(r.e.state().joints[0].mode, Mode::CSV);
  EXPECT_GT(r.e.state().joints[0].velocity, 0);
}

TEST(Control, MovingModeSwitchRejected) {
  Rig r;
  r.start();
  auto cmd = r.commands(Mode::CSV, 0.2);
  for (int i = 0; i < 15; ++i)
    r.step(&cmd);
  cmd = r.commands(Mode::CST, 0.2);
  r.step(&cmd);

  EXPECT_EQ(r.e.state().health.fault, Fault::MovingModeSwitch);
}

TEST(Control, ModeTimeout) {
  Rig r;
  r.start();
  r.t.refuse_mode = true;
  auto cmd = r.commands(Mode::CSV, 0.1);
  for (int i = 0; i < 1010; ++i)
    r.step(&cmd);
  EXPECT_EQ(r.e.state().health.fault, Fault::ModeTimeout);
}

TEST(Control, InvalidCommandPreservesPreviousValidCommand) {
  Rig r;
  r.start();
  auto cmd = r.commands();
  r.step(&cmd);
  const auto stamp = r.e.state().health.last_command_ns;
  cmd[9].position = 99;
  r.step(&cmd);

  EXPECT_EQ(r.e.state().health.fault, Fault::None);
  EXPECT_EQ(r.e.state().health.rejected_commands, 1u);
  EXPECT_EQ(r.e.state().health.rejected_joint, 9);
  EXPECT_EQ(r.e.state().health.last_command_ns, stamp);

  r.step();
  for (auto out : r.t.last_outputs) {
    EXPECT_EQ(out.control, 15);
    EXPECT_EQ(out.position, 0);
  }
}

TEST(Watchdog, CommandsExpire) {
  Rig r;
  r.start();
  auto cmd = r.commands(Mode::CSV, 0.1);
  r.step(&cmd);
  for (int i = 0; i < 25; ++i)
    r.step();
  EXPECT_EQ(r.e.state().health.fault, Fault::CommandExpired);
}

TEST(Watchdog, OldQueuedCommandRejected) {
  Rig r;
  r.start();
  CommandFrame f{r.commands(), r.now, 1};
  r.now += ms(30);
  r.e.tick(r.now, &f);

  EXPECT_EQ(r.e.state().health.fault, Fault::None);
  EXPECT_EQ(r.e.state().health.rejection, RejectReason::Stale);
}

TEST(Watchdog, BadWkcInvalidatesFeedback) {
  Rig r;
  r.start();
  r.t.bad_wkc_cycles = 1;
  r.step();

  EXPECT_FALSE(r.e.state().health.feedback_valid);
  EXPECT_FALSE(r.e.state().joints[0].valid);

  r.t.bad_wkc_cycles = 6;
  for (int i = 0; i < 6; ++i)
    r.step();
  EXPECT_EQ(r.e.state().health.fault, Fault::FeedbackLost);
}

TEST(Watchdog, DisconnectNoAutomaticRestart) {
  Rig r;
  r.start();
  auto cmd = r.commands(Mode::CSV, 0.1);
  r.step(&cmd);
  r.t.lost_motor = 12;
  for (int i = 0; i < 8; ++i)
    r.step();
  EXPECT_EQ(r.e.state().health.fault_motor, 12);

  r.t.lost_motor = -1;
  for (int i = 0; i < 350; ++i)
    r.step(&cmd);
  EXPECT_EQ(r.e.state().health.phase, Phase::Fault);

  for (auto v : r.t.values)
    EXPECT_FALSE(enabled(v.status));
  ASSERT_TRUE(r.e.reset_fault());
  EXPECT_EQ(r.e.state().health.phase, Phase::Inactive);

  r.step(&cmd);

  EXPECT_EQ(r.e.state().health.phase, Phase::Inactive);
}

TEST(Watchdog, DriveFaultLatch) {
  Rig r;
  r.start();
  r.t.values[4].error = 0xa000;
  r.step();

  EXPECT_EQ(r.e.state().health.fault, Fault::DriveFault);
  EXPECT_EQ(r.e.state().health.fault_motor, 4);

  for (int i = 0; i < 350; ++i)
    r.step();
  EXPECT_FALSE(r.e.reset_fault());
}

TEST(Handoff, CommandOverflowAndOrder) {
  CommandFrame f;
  f.joints.resize(15);
  SpscQueue<CommandFrame, 4> q(f);
  for (int i = 1; i < 4; ++i) {
    f.generation = i;

    ASSERT_TRUE(q.push(f));
  }

  EXPECT_FALSE(q.push(f));

  for (int i = 1; i < 4; ++i) {
    ASSERT_TRUE(q.pop(f));
    EXPECT_EQ(f.generation, unsigned(i));
  }

  EXPECT_FALSE(q.pop(f));
}

TEST(Handoff, ConcurrentSnapshotsAreCoherent) {
  Snapshot s;
  s.joints.resize(15);
  Latest<Snapshot> latest(s);
  std::atomic<bool> done{false};
  std::thread p([&] {
    auto v = s;
    for (int n = 1; n <= 100000; ++n) {
      v.health.cycle = n;
      for (auto &j : v.joints)
        j.position = n;
      latest.publish(v);
    }
    done = true;
  });
  do {
    latest.read(s);
    for (auto j : s.joints)
      EXPECT_EQ(j.position, double(s.health.cycle));
  } while (!done);
  p.join();
  latest.read(s);

  EXPECT_EQ(s.health.cycle, 100000u);
}

TEST(Tools, TypedReadAndAllowlistedGains) {
  MockTransport t;
  auto c = config();
  t.discover(c);

  EXPECT_EQ(t.read_object(2, 0x6502, 0, 4), 0x380u);

  t.write_gain(2, 0x2381, 1, 42);

  EXPECT_EQ(t.read_object(2, 0x2381, 1, 2), 42u);
  EXPECT_THROW(t.write_gain(2, 0x2242, 0, 1), std::runtime_error);
  EXPECT_THROW(t.read_object(2, 0xffff, 0, 4), std::runtime_error);
}

TEST(Driver, ThreadedConfigureActivateCommandsShutdown) {
  auto c = config();
  c.lateness_ns = ms(100);
  Driver d(c);
  d.configure();

  ASSERT_TRUE(d.activate());

  Snapshot s;
  s.joints.resize(15);
  std::vector<Command> cmd(15);
  for (auto &j : cmd) {
    j.claimed = true;
    j.position = 0.02;
  }
  for (int i = 0; i < 30; ++i) {
    ASSERT_TRUE(d.submit(cmd));

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    d.read(s);
  }

  EXPECT_EQ(s.health.phase, Phase::Active);
  EXPECT_NEAR(s.joints[14].position, 0.02, 2e-5);
  ASSERT_TRUE(d.deactivate());

  d.shutdown();
}
} // namespace

TEST(Lifecycle, DiscardCommandsFromPreviousActivation) {
  Rig r;
  r.start();
  CommandFrame f{r.commands(Mode::CSP, 0.5), r.now, 1};
  r.e.deactivate(r.now);
  for (int i = 0; i < 310; ++i)
    r.step();
  r.start();
  r.e.tick(r.now, &f);

  EXPECT_EQ(r.e.state().health.phase, Phase::Active);

  for (const auto &s : r.e.state().joints)
    EXPECT_EQ(s.position, 0);
}

TEST(Discovery, PhysicalPortsIncludeNonDcInfrastructure) {
  std::vector<PortNode> nodes{{0, 0, 0}, {0, 15, 0}};
  for (int branch = 0; branch < 3; ++branch) {
    const int count = branch == 0 ? 7 : (branch == 1 ? 5 : 3);
    uint16_t parent = 1;
    for (int i = 0; i < count; ++i) {
      nodes.push_back({parent, uint8_t(i + 1 < count ? 3 : 1), 0});
      parent = uint16_t(nodes.size() - 1);
    }
  }
  auto paths = physical_paths(nodes);

  EXPECT_EQ(paths[2], (std::vector<uint8_t>{3}));
  EXPECT_EQ(paths[9], (std::vector<uint8_t>{1}));
  EXPECT_EQ(paths[14], (std::vector<uint8_t>{2}));
  EXPECT_EQ(paths[8].size(), 7u);

  // An extra infrastructure slave between the junction and a drive is retained.
  nodes = {{0, 0, 0}, {0, 15, 0}, {1, 3, 0}, {2, 1, 0}, {1, 1, 0}, {1, 1, 0}};
  paths = physical_paths(nodes);

  EXPECT_EQ(paths[3], (std::vector<uint8_t>{3, 1}));
  EXPECT_EQ(paths[4], (std::vector<uint8_t>{1}));

  nodes[2].parent = 4;

  EXPECT_THROW(physical_paths(nodes), std::runtime_error);
}

TEST(Pdo, SharedMappingWritesReadBackAndWidths) {
  auto c = config();
  MockTransport t;
  Engine e(c, t);
  e.configure();

  EXPECT_EQ(t.read_object(2, 0x1c12, 1, 2), 0x1600u);
  EXPECT_EQ(t.read_object(2, 0x1600, 6, 4), 0x00000008u);
  EXPECT_EQ(t.read_object(2, 0x1a00, 7, 4), 0x00000008u);
  EXPECT_EQ(t.read_object(2, 0x605a, 0, 2), 2u);
  EXPECT_THROW(t.read_object(2, 0x1600, 6, 2), std::runtime_error);

  MockTransport corrupt;
  corrupt.corrupt_readback = true;
  Engine bad(c, corrupt);

  EXPECT_THROW(bad.configure(), std::runtime_error);
}

TEST(Watchdog, StaleFeedbackWithValidWkc) {
  Rig r;
  r.start();
  r.t.freeze_feedback = true;
  r.step();

  EXPECT_FALSE(r.e.state().health.feedback_valid);

  for (int i = 0; i < 5; ++i)
    r.step();
  EXPECT_EQ(r.e.state().health.fault, Fault::FeedbackLost);
}

TEST(Watchdog, ConsecutiveDeadlineMissesStopGroup) {
  Rig r;
  r.start();
  r.e.monitor_deadline(r.now + ms(4), r.now);
  r.e.monitor_deadline(r.now, r.now);

  EXPECT_EQ(r.e.state().health.phase, Phase::Active);

  for (int i = 0; i < 5; ++i)
    r.e.monitor_deadline(r.now + ms(4), r.now);
  EXPECT_EQ(r.e.state().health.fault, Fault::DeadlineMissed);
  EXPECT_EQ(r.e.state().health.missed_deadlines, 6u);
}

TEST(Driver, ControllerCatchupBurstUsesLatestValidatedSnapshot) {
  auto c = config();
  c.lateness_ns = ms(100);
  Driver d(c);
  d.configure();

  ASSERT_TRUE(d.activate());

  std::vector<Command> commands(15);
  for (auto &command : commands) {
    command.claimed = true;
    command.position = 0.02;
  }
  // A lifecycle callback can pause the controller loop while the bus keeps
  // cycling. Its next updates arrive faster than one command per bus cycle.
  for (int i = 0; i < 1000; ++i)
    ASSERT_TRUE(d.submit(commands));
  Snapshot state;
  state.joints.resize(15);
  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(d.submit(commands));

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    d.read(state);
  }

  EXPECT_EQ(state.health.phase, Phase::Active);
  EXPECT_NEAR(state.joints[0].position, 0.02, 2e-5);
  ASSERT_TRUE(d.deactivate());
}

TEST(Driver, RejectionIsReportedWithoutFaultOrHidingLaterValidSubmission) {
  auto c = config();
  c.lateness_ns = ms(100);
  Driver d(c);
  d.configure();

  ASSERT_TRUE(d.activate());

  std::vector<Command> commands(15);
  for (auto &command : commands)
    command.claimed = true;
  commands[4].position = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(d.submit(commands));

  commands[4].position = 0;

  EXPECT_TRUE(d.submit(commands));

  Snapshot state;
  state.joints.resize(15);
  for (int i = 0; i < 20; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    d.read(state);

    ASSERT_TRUE(d.submit(commands));
  }

  EXPECT_EQ(state.health.fault, Fault::None);
  EXPECT_EQ(state.health.rejected_commands, 1u);
  EXPECT_EQ(state.health.rejected_joint, 4);
}

namespace {
BusConfig hold_config() {
  auto c = config();
  c.csp_hold_enabled = true;
  c.lateness_ns = ms(4);
  c.feedback_timeout_ns = ms(10);
  return c;
}

struct HoldRig {
  BusConfig c;
  MockTransport t;
  Engine e;
  Nanoseconds now = ms(1);
  uint64_t generation = 0;

  explicit HoldRig(BusConfig cfg = hold_config()) : c(cfg), e(c, t) {
    e.configure();
    e.tick(now);
    if (!e.activate(now))
      throw std::runtime_error("activate failed");
    for (int i = 0; i < 510; ++i)
      step();
  }

  void step(const std::vector<Command> *commands = nullptr, bool late = false,
            Nanoseconds dt = ms(1)) {
    now += dt;
    e.monitor_deadline(now, late ? now - c.lateness_ns - 1 : now);
    CommandFrame f;
    if (commands)
      f = {*commands, now, ++generation};
    e.tick(now, commands ? &f : nullptr);
  }

  std::vector<Command> command(bool claimed = true) {
    std::vector<Command> out(15);
    for (size_t i = 0; i < out.size(); ++i) {
      out[i].claimed = claimed;
      out[i].position = e.state().joints[i].position;
    }
    return out;
  }

  void expire() {
    auto cmd = command();
    step(&cmd);
    for (int i = 0; i < 20; ++i)
      step();
  }

  void settle() {
    for (int i = 0; i < 120; ++i)
      step();
  }
};

TEST(Hold, StationaryLatchFreshCommandsCannotResumeAndNormalDisable) {
  HoldRig r;
  r.expire();

  EXPECT_EQ(r.e.state().health.phase, Phase::Decelerating);

  r.settle();

  ASSERT_EQ(r.e.state().health.phase, Phase::Holding);

  auto cmd = r.command();
  for (auto &c : cmd)
    c.position = .3;
  for (int i = 0; i < 150; ++i)
    r.step(&cmd);
  for (auto out : r.t.last_outputs) {
    EXPECT_EQ(out.control, 15);
    EXPECT_EQ(out.position, 0);
  }

  EXPECT_FALSE(r.e.resume(r.e.state().health.hold_generation, r.now));

  r.e.deactivate(r.now);
  for (int i = 0; i < 305; ++i)
    r.step();
  EXPECT_EQ(r.e.state().health.phase, Phase::Inactive);
}

TEST(Hold, DeadlineThresholdsAreFiveAndFifteenTotal) {
  HoldRig r;
  for (int i = 0; i < 4; ++i)
    r.step(nullptr, true);
  EXPECT_EQ(r.e.state().health.phase, Phase::Active);

  r.step(nullptr, true);

  EXPECT_TRUE(holding(r.e.state().health.phase));

  for (int i = 0; i < 9; ++i)
    r.step(nullptr, true);
  EXPECT_NE(r.e.state().health.phase, Phase::Fault);

  r.step(nullptr, true);

  EXPECT_EQ(r.e.state().health.fault, Fault::DeadlineMissed);
  EXPECT_EQ(r.e.state().health.late_cycles, 15u);
}

TEST(Hold, DeadlineCounterResetsButHoldDoesNot) {
  HoldRig r;
  for (int i = 0; i < 5; ++i)
    r.step(nullptr, true);
  r.step();

  EXPECT_EQ(r.e.state().health.late_cycles, 0u);

  for (int i = 0; i < 14; ++i)
    r.step(nullptr, true);
  EXPECT_TRUE(holding(r.e.state().health.phase));

  r.step();
  r.settle();

  EXPECT_EQ(r.e.state().health.phase, Phase::Holding);
}

TEST(Hold, FeedbackThresholdsFrozenTargetsRecoveryAndFifteenTotal) {
  HoldRig r;
  auto cmd = r.command();
  r.step(&cmd);
  r.t.bad_wkc_cycles = 5;
  for (int i = 0; i < 5; ++i)
    r.step(&cmd);
  EXPECT_EQ(r.e.state().health.phase, Phase::HoldUnconfirmed);
  EXPECT_FALSE(r.e.state().health.feedback_valid);

  for (auto out : r.t.last_outputs) {
    EXPECT_EQ(out.control, 15);
    EXPECT_EQ(out.position, 0);
  }
  r.step(&cmd);
  r.settle();

  EXPECT_EQ(r.e.state().health.phase, Phase::Holding);
  EXPECT_EQ(r.e.state().health.bad_cycles, 0u);

  r.t.bad_wkc_cycles = 15;
  for (int i = 0; i < 14; ++i)
    r.step();
  EXPECT_EQ(r.e.state().health.phase, Phase::HoldUnconfirmed);

  r.step();

  EXPECT_EQ(r.e.state().health.fault, Fault::FeedbackLost);
}

TEST(Hold, FeedbackElapsedTimeThresholdIndependentOfCycleCount) {
  auto cfg = hold_config();
  cfg.bad_cycles = 100;
  HoldRig r(cfg);
  r.t.freeze_feedback = true;
  r.step(nullptr, false, ms(10));

  EXPECT_EQ(r.e.state().health.phase, Phase::HoldUnconfirmed);

  r.step(nullptr, false, ms(19));

  EXPECT_NE(r.e.state().health.phase, Phase::Fault);

  r.step(nullptr, false, ms(1));

  EXPECT_EQ(r.e.state().health.fault, Fault::FeedbackLost);
}

TEST(Hold, ExplicitResumeRequiresResetFreshnessAndCorrectGeneration) {
  HoldRig r;
  r.expire();
  r.settle();
  const auto gen = r.e.state().health.hold_generation;
  auto cmd = r.command();
  r.step(&cmd);

  EXPECT_FALSE(r.e.resume(gen, r.now)); // claims were never released

  cmd = r.command(false);
  r.step(&cmd);
  cmd = r.command();
  r.step(&cmd);

  EXPECT_TRUE(r.e.state().health.resume_ready);
  EXPECT_FALSE(r.e.resume(gen - 1, r.now));

  for (int i = 0; i < 20; ++i)
    r.step();
  EXPECT_FALSE(r.e.resume(gen, r.now));

  r.step(&cmd);

  ASSERT_TRUE(r.e.resume(gen, r.now));
  EXPECT_EQ(r.e.state().health.phase, Phase::Active);

  // A frame sampled before the acknowledgment must not be accepted after resume.
  CommandFrame old{cmd, r.now, ++r.generation};
  old.joints[0].position = .4;
  r.e.tick(r.now + 1, &old);

  EXPECT_EQ(r.e.state().health.rejection, RejectReason::Stale);

  cmd[0].position = .01;
  r.step(&cmd);
  r.step(&cmd);

  EXPECT_NEAR(r.e.state().joints[0].position, .01, 2e-5);
}

TEST(Hold, RejectionDoesNotRefreshTimestampOrPoisonGeneration) {
  HoldRig r;
  auto good = r.command();
  r.step(&good);
  auto stamp = r.e.state().health.last_command_ns;
  auto bad = good;
  bad[7].position = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 20; ++i)
    r.step(&bad);
  EXPECT_EQ(r.e.state().health.last_command_ns, stamp);
  EXPECT_EQ(r.e.state().health.rejected_commands, 20u);
  EXPECT_TRUE(holding(r.e.state().health.phase));

  r.step(&good);

  EXPECT_TRUE(holding(r.e.state().health.phase));
  EXPECT_EQ(r.e.state().health.fault, Fault::None);

  CommandFrame future{good, r.now + ms(100), UINT64_MAX};
  r.e.tick(r.now, &future);
  r.step(&good);

  EXPECT_EQ(r.e.state().health.last_command_ns, r.now);
}

TEST(Hold, MovingProfileIsBoundedAndHasNoReversalAllJoints) {
  HoldRig r;
  auto cmd = r.command();
  // Ramp both directions while causing exactly five consecutive late wakeups.
  for (int k = 0; k < 10; ++k) {
    for (size_t j = 0; j < cmd.size(); ++j)
      cmd[j].position = (j % 2 ? -1 : 1) * .0001 * k;
    r.step(&cmd, k >= 5);
  }

  ASSERT_EQ(r.e.state().health.phase, Phase::Decelerating);

  const auto start = r.e.state();
  std::vector<double> previous(15);
  double T = 0;
  for (int j = 0; j < 15; ++j) {
    const double v = std::abs(start.joints[j].velocity);
    T = std::max(
        {T, 1.5 * v / r.c.drives[j].hold_acceleration, std::sqrt(6 * v / r.c.drives[j].hold_jerk)});
  }
  T = std::ceil(T * 1000) / 1000;
  for (int j = 0; j < 15; ++j) {
    EXPECT_LE(1.5 * std::abs(start.joints[j].velocity) / T, r.c.drives[j].hold_acceleration + 1e-9);
    EXPECT_LE(6 * std::abs(start.joints[j].velocity) / (T * T), r.c.drives[j].hold_jerk + 1e-9);
    EXPECT_NEAR(start.joints[j].hold_target,
                start.joints[j].hold_start + start.joints[j].velocity * T / 2, 1e-10);
  }
  for (int j = 0; j < 15; ++j)
    previous[j] = start.joints[j].position;
  for (int k = 0; k < 500; ++k) {
    r.step();

    ASSERT_NE(r.e.state().health.phase, Phase::Fault);

    for (int j = 0; j < 15; ++j) {
      const double q = joint_position(r.t.last_outputs[j].position, r.c.drives[j]);
      const double u = std::min(k * .001 / T, 1.);
      const double expected = start.joints[j].hold_start +
                              start.joints[j].velocity * T * (u - u * u * u + .5 * u * u * u * u);
      EXPECT_NEAR(q, expected, 1.0 / r.c.drives[j].position_counts_per_rad);
      EXPECT_GE((q - previous[j]) * (j % 2 ? -1 : 1), -2e-5);
      EXPECT_LE(std::abs(q - previous[j]) / .001, .13);

      previous[j] = q;
    }
  }

  ASSERT_EQ(r.e.state().health.phase, Phase::Holding);

  for (int j = 0; j < 15; ++j) {
    EXPECT_NEAR(r.e.state().joints[j].position, start.joints[j].hold_target, 2e-5);
    EXPECT_GT((start.joints[j].hold_target - start.joints[j].hold_start) * (j % 2 ? -1 : 1), 0);
    EXPECT_EQ(r.t.last_outputs[j].control, 15);
  }
}

TEST(Hold, FeedbackGapDuringDecelerationFreezesAndReplans) {
  HoldRig r;
  auto cmd = r.command();
  for (int k = 0; k < 10; ++k) {
    cmd[0].position = k * .0001;
    r.step(&cmd, k >= 5);
  }

  ASSERT_EQ(r.e.state().health.phase, Phase::Decelerating);

  r.t.bad_wkc_cycles = 3;
  r.step();
  const auto q = r.t.last_outputs[0].position;

  EXPECT_EQ(r.e.state().health.phase, Phase::HoldUnconfirmed);

  r.step();
  r.step();

  EXPECT_EQ(r.t.last_outputs[0].position, q);

  const auto generation = r.e.state().health.hold_generation;
  r.step();

  EXPECT_EQ(r.e.state().health.phase, Phase::Decelerating);
  EXPECT_EQ(r.e.state().health.hold_generation, generation);

  for (int k = 0; k < 600; ++k)
    r.step();
  EXPECT_EQ(r.e.state().health.phase, Phase::Holding);
}

TEST(Hold, RejectWatchdogDisabledDriveOnFeedbackReturn) {
  HoldRig r;
  r.t.bad_wkc_cycles = 5;
  for (int k = 0; k < 5; ++k)
    r.step();
  r.t.freeze_enable = true;
  r.t.values[0].status = 0x40;
  r.step();

  EXPECT_EQ(r.e.state().health.fault, Fault::DriveFault);
}

TEST(Hold, TrackingViolationAndInfeasibleStoppingPath) {
  HoldRig r;
  r.expire();
  r.settle();
  r.t.freeze_motion = true;
  r.t.values[0].position = 100000;
  r.step();

  EXPECT_EQ(r.e.state().health.fault, Fault::HoldFailed);

  auto c = hold_config();
  c.drives[0].max_position = .01;
  HoldRig edge(c);
  edge.t.freeze_motion = true;
  edge.t.values[0].velocity = int32_t(.5 * c.drives[0].velocity_counts_per_rad_s);
  for (int k = 0; k < 5; ++k)
    edge.step(nullptr, true);
  EXPECT_EQ(edge.e.state().health.fault, Fault::HoldFailed);
}

TEST(Hold, FailedSettlingEscalatesButHealthyHoldDoesNotExpire) {
  HoldRig r;
  r.t.freeze_motion = true;
  r.t.values[0].velocity = int32_t(.02 * r.c.drives[0].velocity_counts_per_rad_s);
  for (int k = 0; k < 5; ++k)
    r.step(nullptr, true);
  for (int k = 0; k < 11000 && r.e.state().health.phase != Phase::Fault; ++k)
    r.step();
  EXPECT_EQ(r.e.state().health.fault, Fault::HoldFailed);

  HoldRig healthy;
  healthy.expire();
  for (int k = 0; k < 20000; ++k)
    healthy.step();
  EXPECT_EQ(healthy.e.state().health.phase, Phase::Holding);
}

TEST(Hold, RealHoldProfileGateAndDiscoveryRemainSeparate) {
  auto c = hold_config();
  c.backend = "soem";
  c.interface = "fake0";
  c.topology_verified = c.stop_verified = c.watchdog_verified = c.padding_verified = true;
  c.hub_model = "test";
  c.esi_reference = "test";
  for (auto &d : c.drives)
    d.verified = true;
  EXPECT_NO_THROW(validate_config(c));
  EXPECT_THROW(validate_activation(c), std::runtime_error);

  for (auto &d : c.drives)
    d.hold_verified = true;
  EXPECT_NO_THROW(validate_activation(c));
}
} // namespace

TEST(Driver, ResumeAcknowledgmentAndConcurrentShutdown) {
  auto cfg = hold_config();
  cfg.lateness_ns = ms(100);
  Driver d(cfg);
  d.configure();

  ASSERT_TRUE(d.activate());

  std::vector<Command> cmd(15);
  for (auto &c : cmd)
    c.claimed = true;
  ASSERT_TRUE(d.submit(cmd));

  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  Snapshot s;
  s.joints.resize(15);
  d.read(s);

  ASSERT_EQ(s.health.phase, Phase::Holding);

  auto gen = s.health.hold_generation;

  EXPECT_FALSE(d.resume(gen));

  for (auto &c : cmd)
    c.claimed = false;
  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(d.submit(cmd));

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  for (auto &c : cmd)
    c.claimed = true;
  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(d.submit(cmd));

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  ASSERT_TRUE(d.resume(gen));
  ASSERT_TRUE(d.submit(cmd));

  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  d.read(s);

  ASSERT_EQ(s.health.phase, Phase::Holding);
  EXPECT_FALSE(d.resume(gen)); // Previous hold generation.

  std::thread resumer([&] { d.resume(s.health.hold_generation); });
  d.shutdown();
  resumer.join();
  d.read(s);

  EXPECT_EQ(s.health.phase, Phase::Inactive);
  EXPECT_EQ(d.submit(cmd).code, SubmitResult::Unavailable);
}

TEST(Hold, ResumeStillExpiresIfControllerStopsImmediatelyAfterAcknowledgment) {
  HoldRig r;
  r.expire();
  r.settle();
  auto cmd = r.command(false);
  r.step(&cmd);
  cmd = r.command();
  r.step(&cmd);
  const auto generation = r.e.state().health.hold_generation;

  ASSERT_TRUE(r.e.resume(generation, r.now));

  for (int i = 0; i < 20; ++i)
    r.step();
  EXPECT_TRUE(holding(r.e.state().health.phase));
  EXPECT_EQ(r.e.state().health.hold_generation, generation + 1);
  EXPECT_EQ(r.e.state().health.hold_reason, Fault::CommandExpired);
}

TEST(Hold, LongHoldDisturbanceGetsSettlingTimeWithoutMovingReference) {
  HoldRig r;
  r.expire();
  for (int i = 0; i < 12000; ++i)
    r.step();
  ASSERT_EQ(r.e.state().health.phase, Phase::Holding);

  const auto target = r.e.state().joints[0].hold_target;
  r.t.freeze_motion = true;
  r.t.values[0].velocity = int32_t(.02 * r.c.drives[0].velocity_counts_per_rad_s);
  r.step();

  EXPECT_EQ(r.e.state().health.phase, Phase::Decelerating);
  EXPECT_EQ(r.e.state().health.fault, Fault::None);
  EXPECT_EQ(r.e.state().joints[0].hold_target, target);

  r.t.values[0].velocity = 0;
  r.settle();

  EXPECT_EQ(r.e.state().health.phase, Phase::Holding);
}

TEST(Hold, MalformedOrderModeAndOverflowAreRejectedAsWholeSnapshots) {
  HoldRig r;
  auto good = r.command();
  r.step(&good);
  const auto stamp = r.e.state().health.last_command_ns;
  auto bad = good;
  bad.pop_back();
  r.step(&bad);

  EXPECT_EQ(r.e.state().health.rejection, RejectReason::Shape);

  CommandFrame duplicate{good, r.now, 1};
  r.e.tick(r.now, &duplicate);

  EXPECT_EQ(r.e.state().health.rejection, RejectReason::Order);

  bad = good;
  bad[0].mode = Mode::CSV;
  r.step(&bad);

  EXPECT_EQ(r.e.state().health.rejection, RejectReason::Mode);

  bad = good;
  bad[4].position = std::numeric_limits<double>::max();
  r.step(&bad);

  EXPECT_EQ(r.e.state().health.rejection, RejectReason::Value);
  EXPECT_EQ(r.e.state().health.rejected_commands, 4u);
  EXPECT_EQ(r.e.state().health.last_command_ns, stamp);
  EXPECT_EQ(r.e.state().health.fault, Fault::None);

  for (auto out : r.t.last_outputs)
    EXPECT_EQ(out.position, 0);
  r.step(&good);

  EXPECT_EQ(r.e.state().health.last_command_ns, r.now);
}

namespace {
BusConfig feedforward_config() {
  auto c = hold_config();
  c.csp_feedforward = true;
  for (auto &d : c.drives) {
    d.csp_feedforward = true;
    d.max_velocity_offset = .5;
    d.max_torque_offset = .5;
    d.offset_ramp_seconds = .2;
  }
  return c;
}

TEST(Feedforward, SignedDistinctObjectsAndCalibration) {
  auto c = feedforward_config();
  auto d = c.drives.front();
  d.direction = -1;
  Command command{Mode::CSP, .02, 0, 0, true, .15, -.12};

  ASSERT_TRUE(command_valid(command, d));

  RxPdo p;
  apply_command(command, d, p);
  const auto bytes = encode(p);
  const auto received = decode(bytes);

  EXPECT_EQ(received.velocity, 0);
  EXPECT_EQ(received.torque, 0);
  EXPECT_NEAR(joint_velocity(received.velocity_offset, d), .15, 1 / d.velocity_counts_per_rad_s);
  EXPECT_NEAR(joint_effort(received.torque_offset, d), -.12, d.nm_per_raw);
  EXPECT_EQ(bytes[18], uint8_t(120));
  EXPECT_EQ(bytes[19], 0);

  d.direction = 1;
  apply_command(command, d, p);
  const auto negative = encode(p);

  EXPECT_EQ(negative[18], uint8_t(-120));
  EXPECT_EQ(negative[19], 255);

  command.mode = Mode::CSV;

  EXPECT_FALSE(command_valid(command, d));

  command.mode = Mode::CST;

  EXPECT_FALSE(command_valid(command, d));

  command.mode = Mode::CSP;
  d.csp_feedforward = false;

  EXPECT_FALSE(command_valid(command, d));

  d.csp_feedforward = true;
  d.effort_verified = false;

  EXPECT_FALSE(command_valid(command, d));
}

TEST(Feedforward, ExtendedMappingReadbackAndImageSize) {
  auto c = feedforward_config();
  MockTransport t;
  Engine e(c, t);
  e.configure();

  EXPECT_EQ(t.read_object(2, 0x1600, 0, 1), 8u);
  EXPECT_EQ(t.read_object(2, 0x1600, 7, 4), 0x60b10020u);
  EXPECT_EQ(t.read_object(2, 0x1600, 8, 4), 0x60b20010u);
  EXPECT_EQ(t.read_object(2, 0x1600, 3, 4), 0x60ff0020u);
  EXPECT_EQ(t.read_object(2, 0x1600, 4, 4), 0x60710010u);
  EXPECT_EQ(rx_wire_size(true), 20u);

  MockTransport corrupt;
  corrupt.corrupt_readback = true;
  Engine a(c, corrupt);

  EXPECT_THROW(a.configure(), std::runtime_error);

  MockTransport wrong;
  wrong.wrong_size = true;
  Engine b(c, wrong);

  EXPECT_THROW(b.configure(), std::runtime_error);

  c.drives[0].nm_per_raw = 0;

  EXPECT_THROW(validate_config(c), std::runtime_error);
}

TEST(Feedforward, TransportWatchdogRampAndNoAutomaticResume) {
  HoldRig r(feedforward_config());
  auto cmd = r.command();
  cmd[0].torque_offset = .3;
  cmd[0].velocity_offset = -.1;
  for (int i = 0; i < 10; ++i)
    r.step(&cmd);
  EXPECT_EQ(r.t.last_outputs[0].torque_offset, 300);
  EXPECT_LT(r.t.last_outputs[0].velocity_offset, 0);

  const auto rejected = r.e.state().health.rejected_commands;
  cmd[0].torque_offset = std::numeric_limits<double>::quiet_NaN();
  for (int i = 0; i < 30; ++i)
    r.step(&cmd);
  EXPECT_GT(r.e.state().health.rejected_commands, rejected);
  EXPECT_TRUE(holding(r.e.state().health.phase));
  EXPECT_GT(r.t.last_outputs[0].torque_offset, 0);

  int previous = r.t.last_outputs[0].torque_offset;
  for (int i = 0; i < 350; ++i) {
    r.step();

    EXPECT_LE(r.t.last_outputs[0].torque_offset, previous);

    previous = r.t.last_outputs[0].torque_offset;
  }

  EXPECT_EQ(r.e.state().health.phase, Phase::Holding);
  EXPECT_EQ(r.t.last_outputs[0].torque_offset, 0);
  EXPECT_EQ(r.t.last_outputs[0].velocity_offset, 0);

  cmd = r.command();
  cmd[0].torque_offset = .1;
  for (int i = 0; i < 50; ++i)
    r.step(&cmd);
  EXPECT_EQ(r.t.last_outputs[0].torque_offset, 0);
  EXPECT_FALSE(r.e.resume(r.e.state().health.hold_generation, r.now));

  r.e.deactivate(r.now);
  for (int i = 0; i < 310; ++i)
    r.step();
  EXPECT_EQ(r.e.state().health.phase, Phase::Inactive);
  EXPECT_EQ(r.t.last_outputs[0].torque_offset, 0);
}

TEST(Feedforward, DisableAndFaultClearOffsets) {
  HoldRig r(feedforward_config());
  auto cmd = r.command();
  cmd[0].torque_offset = -.2;
  for (int i = 0; i < 5; ++i)
    r.step(&cmd);
  r.e.deactivate(r.now);
  r.step();
  r.step();

  EXPECT_EQ(r.t.last_outputs[0].torque_offset, 0);

  HoldRig f(feedforward_config());
  cmd = f.command();
  cmd[0].torque_offset = .2;
  for (int i = 0; i < 5; ++i)
    f.step(&cmd);
  f.t.values[0].error = 0x2310;
  for (int i = 0; i < 5; ++i)
    f.step(&cmd);
  EXPECT_NE(f.e.state().health.phase, Phase::Active);
  EXPECT_EQ(f.t.last_outputs[0].torque_offset, 0);
}
}
