#include <chrono>
#include <gtest/gtest.h>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <thread>

namespace {
std::string urdf() {
  std::ostringstream s;
  s << "<robot name='test'><link name='base'/><ros2_control name='ErobSystem' "
       "type='system'><hardware><plugin>erob_hardware/ErobSystem</"
       "plugin><param name='config_file'>"
    << EROB_TEST_CONFIG << "</param><param name='backend'>mock</param></hardware>";
  for (int i = 1; i <= 15; ++i) {
    s << "<joint name='motor_" << std::hex << (i < 16 ? "0" : "") << i << "_joint'>";
    s << "<command_interface name=\"position\"/>";
    for (const auto &name : {"position", "velocity", "effort"})
      s << "<state_interface name=\"" << name << "\"/>";
    s << "</joint>";
  }
  s << "</ros2_control></robot>";
  return s.str();
}

class HardwareTest : public testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }
};

TEST_F(HardwareTest, PluginLoadsAndResourceManagerValidates) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
  EXPECT_NO_THROW(loader.createSharedInstance("erob_hardware/ErobSystem"));
  EXPECT_NO_THROW({
    hardware_interface::ResourceManager manager(urdf(), true, false);

    EXPECT_EQ(manager.command_interface_keys().size(), 15u);
    EXPECT_EQ(manager.state_interface_keys().size(), 45u);
  });
}

TEST_F(HardwareTest, RejectUrdfMismatch) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
  auto instance = loader.createSharedInstance("erob_hardware/ErobSystem");
  auto &h = *instance;
  auto info = hardware_interface::parse_control_resources_from_urdf(urdf())[0];
  info.joints[1].name = "wrong_joint";

  EXPECT_EQ(h.on_init(info), hardware_interface::CallbackReturn::ERROR);
}

TEST_F(HardwareTest, LifecycleClaimsAndCommands) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
  auto instance = loader.createSharedInstance("erob_hardware/ErobSystem");
  auto &h = *instance;
  auto info = hardware_interface::parse_control_resources_from_urdf(urdf())[0];

  ASSERT_EQ(h.on_init(info), hardware_interface::CallbackReturn::SUCCESS);

  auto state = h.export_state_interfaces();
  auto command = h.export_command_interfaces();
  rclcpp_lifecycle::State unused;

  ASSERT_EQ(h.on_configure(unused), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(h.on_activate(unused), hardware_interface::CallbackReturn::SUCCESS);

  std::vector<std::string> position, velocity;
  for (const auto &j : info.joints) {
    position.push_back(j.name + "/position");
    velocity.push_back(j.name + "/velocity");
  }

  ASSERT_EQ(h.prepare_command_mode_switch(position, {}), hardware_interface::return_type::OK);
  ASSERT_EQ(h.perform_command_mode_switch(position, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(h.prepare_command_mode_switch(velocity, {}), hardware_interface::return_type::ERROR);

  for (auto &c : command)
    if (c.get_interface_name() == "position")
      c.set_value(0.04);
  for (int i = 0; i < 20; ++i) {
    h.read(rclcpp::Time(0), rclcpp::Duration(0, 1000000));

    ASSERT_EQ(h.write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
              hardware_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  h.read(rclcpp::Time(0), rclcpp::Duration(0, 1000000));
  for (const auto &s : state)
    if (s.get_interface_name() == "position")
      EXPECT_NEAR(s.get_value(), 0.04, 2e-5);
  ASSERT_EQ(h.prepare_command_mode_switch(velocity, position),
            hardware_interface::return_type::ERROR);
  ASSERT_EQ(h.prepare_command_mode_switch({}, position), hardware_interface::return_type::OK);
  ASSERT_EQ(h.perform_command_mode_switch({}, position), hardware_interface::return_type::OK);
  ASSERT_EQ(h.on_deactivate(unused), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(h.on_cleanup(unused), hardware_interface::CallbackReturn::SUCCESS);
}
} // namespace

TEST_F(HardwareTest, RejectsNonPositionClaimsAndRealFaultInjection) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
  auto instance = loader.createSharedInstance("erob_hardware/ErobSystem");
  auto info = hardware_interface::parse_control_resources_from_urdf(urdf())[0];
  auto bad = info;
  bad.joints[0].command_interfaces[0].name = "velocity";

  EXPECT_EQ(instance->on_init(bad), hardware_interface::CallbackReturn::ERROR);

  instance = loader.createSharedInstance("erob_hardware/ErobSystem");
  info.hardware_parameters["backend"] = "soem";
  info.hardware_parameters["enable_mock_fault_injection"] = "true";

  EXPECT_EQ(instance->on_init(info), hardware_interface::CallbackReturn::ERROR);
}

TEST_F(HardwareTest, InvalidCommandsDoNotPropagateRosHardwareError) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
  auto h = loader.createSharedInstance("erob_hardware/ErobSystem");
  auto info = hardware_interface::parse_control_resources_from_urdf(urdf())[0];

  ASSERT_EQ(h->on_init(info), hardware_interface::CallbackReturn::SUCCESS);

  rclcpp_lifecycle::State unused;

  ASSERT_EQ(h->on_configure(unused), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(h->on_activate(unused), hardware_interface::CallbackReturn::SUCCESS);

  const std::vector<std::string> keys{"motor_01_joint/position"};

  ASSERT_EQ(h->prepare_command_mode_switch(keys, {}), hardware_interface::return_type::OK);
  ASSERT_EQ(h->perform_command_mode_switch(keys, {}), hardware_interface::return_type::OK);

  auto commands = h->export_command_interfaces();
  h->write(rclcpp::Time(0), rclcpp::Duration(0, 1000000));
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  commands[0].set_value(999);
  for (int i = 0; i < 150; ++i) {
    ASSERT_EQ(h->write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
              hardware_interface::return_type::OK);
    ASSERT_EQ(h->read(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
              hardware_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  ASSERT_EQ(h->on_deactivate(unused), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(h->on_cleanup(unused), hardware_interface::CallbackReturn::SUCCESS);
}

#include <fstream>
#include <filesystem>
#include <unistd.h>

TEST_F(HardwareTest, FeedforwardInterfacesAreClaimedTogether) {
  std::ifstream source(EROB_TEST_CONFIG);
  std::string config((std::istreambuf_iterator<char>(source)), std::istreambuf_iterator<char>());
  config.insert(config.find("bus:\n") + 5, "  csp_feedforward: true\n");
  config.insert(
      config.find("profile:\n") + 9,
      "  max_velocity_offset: 1.0\n  max_torque_offset: 0.5\n  offset_ramp_seconds: 0.2\n");
  const auto path = std::filesystem::temp_directory_path() /
                    ("erob_ff_test_" + std::to_string(getpid()) + ".yaml");
  {
    std::ofstream file(path);
    file << config;
  }
  auto info = hardware_interface::parse_control_resources_from_urdf(urdf())[0];
  info.hardware_parameters["config_file"] = path.string();
  for (auto &joint : info.joints) {
    auto interface = joint.command_interfaces.front();
    interface.name = "velocity_offset";
    joint.command_interfaces.push_back(interface);
    interface.name = "torque_offset";
    joint.command_interfaces.push_back(interface);
  }
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
  auto h = loader.createSharedInstance("erob_hardware/ErobSystem");
  const auto initialized = h->on_init(info);
  std::filesystem::remove(path);

  ASSERT_EQ(initialized, hardware_interface::CallbackReturn::SUCCESS);

  auto interfaces = h->export_command_interfaces();

  EXPECT_EQ(interfaces.size(), 45u);

  const std::string name = "motor_01_joint/";

  EXPECT_EQ(h->prepare_command_mode_switch({name + "velocity_offset"}, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(h->prepare_command_mode_switch({name + "position"}, {}),
            hardware_interface::return_type::ERROR);
  std::vector<std::string> bundle{name + "position", name + "velocity_offset",
                                  name + "torque_offset"};
  ASSERT_EQ(h->prepare_command_mode_switch(bundle, {}), hardware_interface::return_type::OK);
  ASSERT_EQ(h->perform_command_mode_switch(bundle, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(h->prepare_command_mode_switch({}, {name + "position"}),
            hardware_interface::return_type::ERROR);
  ASSERT_EQ(h->prepare_command_mode_switch({}, bundle), hardware_interface::return_type::OK);
  ASSERT_EQ(h->perform_command_mode_switch({}, bundle), hardware_interface::return_type::OK);

  for (const auto &i : interfaces)
    if (i.get_interface_name() != "position")
      EXPECT_EQ(i.get_value(), 0);
}
