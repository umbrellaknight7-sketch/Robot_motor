#pragma once

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <erob_driver/driver.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace erob_hardware {
class ErobSystem final : public hardware_interface::SystemInterface {
public:
  ~ErobSystem() override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type
  prepare_command_mode_switch(const std::vector<std::string> &,
                              const std::vector<std::string> &) override;
  hardware_interface::return_type
  perform_command_mode_switch(const std::vector<std::string> &,
                              const std::vector<std::string> &) override;

private:
  void stop() noexcept;
  void diagnostics();
  erob::BusConfig config_;
  std::unique_ptr<erob::Driver> driver_;
  erob::Snapshot snapshot_, switch_snapshot_;
  std::unique_ptr<erob::Latest<erob::Snapshot>> diagnostics_state_, switch_state_;
  std::vector<double> position_, velocity_, effort_, cmd_position_, cmd_velocity_, cmd_effort_,
      cmd_velocity_offset_, cmd_torque_offset_;
  std::vector<erob::Command> commands_;
  std::vector<int> claimed_, pending_;
  bool active_ = false, prepared_ = false;
  std::atomic<bool> diagnostics_running_{false};
  std::thread diagnostics_thread_;
  bool mock_fault_injection_ = false;
  std::atomic<erob::Nanoseconds> pause_commands_until_{0};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_expire_service_;
  uint64_t service_generation_ = 0; // Accessed only by diagnostics executor thread.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;
  rclcpp::Node::SharedPtr diagnostics_node_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};
} // namespace erob_hardware
