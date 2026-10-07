#include "wheel_human_commander/kinematics.hpp"

#include <rclcpp/rclcpp.hpp>
#include <wheel_human_interfaces/srv/compute_fk.hpp>
#include <wheel_human_interfaces/srv/compute_ik.hpp>
#include <memory>
#include <stdexcept>

namespace wheel_human_commander {
class KinematicsNode final : public rclcpp::Node {
public:
  KinematicsNode() : Node("wheel_human_kinematics") {
    auto description = declare_parameter<std::string>("robot_description", "");
    IkOptions options;
    options.max_iterations = declare_parameter<int>("solver.max_iterations", 200);
    options.timeout_seconds = declare_parameter<double>("solver.timeout_seconds", .1);
    options.max_joint_step = declare_parameter<double>("solver.max_joint_step", .1);
    options.position_tolerance = declare_parameter<double>("solver.position_tolerance", .0001);
    options.orientation_tolerance = declare_parameter<double>("solver.orientation_tolerance", .001);
    options.orientation_scale = declare_parameter<double>("solver.orientation_scale", .1);
    options.damping = declare_parameter<double>("solver.damping", .001);
    std::vector<KinematicsGroup> groups;
    auto names =
        declare_parameter<std::vector<std::string>>("planning_groups", {"left_arm", "right_arm"});
    for (const auto &name : names) {
      KinematicsGroup g;
      g.name = name;
      g.joints = declare_parameter<std::vector<std::string>>("groups." + name + ".joints",
                                                             std::vector<std::string>{});
      g.tip = declare_parameter<std::string>("groups." + name + ".tip", "");
      groups.push_back(std::move(g));
    }
    model_ = std::make_unique<KinematicsModel>(description, groups, options);
    RCLCPP_INFO(get_logger(),
                "Pinocchio %s, model %s, root base_link, %zu movable joints; calculations only",
                KinematicsModel::pinocchio_version().c_str(), model_->fingerprint().c_str(),
                model_->joint_names().size());
    for (const auto &g : groups)
      RCLCPP_INFO(get_logger(), "IK group %s: %zu variable joints, TCP %s", g.name.c_str(),
                  g.joints.size(), g.tip.c_str());
    fk_ = create_service<FK>("~/compute_fk", [this](const std::shared_ptr<FK::Request> request,
                                                    std::shared_ptr<FK::Response> response) {
      auto result = model_->forward(request->configuration, request->target_frame);
      response->error_code = static_cast<int>(result.code);
      response->message = result.message;
      response->pose = result.pose;
    });
    ik_ = create_service<IK>("~/compute_ik", [this](const std::shared_ptr<IK::Request> request,
                                                    std::shared_ptr<IK::Response> response) {
      auto result = model_->inverse(request->seed, request->planning_group, request->target,
                                    static_cast<IkMode>(request->target_mode));
      response->error_code = static_cast<int>(result.code);
      response->message = result.message;
      response->solution = result.solution;
      response->achieved_pose = result.pose;
      response->position_error = result.position_error;
      response->orientation_error = result.orientation_error;
      response->iterations = result.iterations;
    });
    // Parameters describe an immutable model/solver. Restart to change them.
    parameters_ = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> &) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = "Kinematics parameters are startup-only; restart the node";
      return result;
    });
  }

private:
  using FK = wheel_human_interfaces::srv::ComputeFK;
  using IK = wheel_human_interfaces::srv::ComputeIK;
  std::unique_ptr<KinematicsModel> model_;
  rclcpp::Service<FK>::SharedPtr fk_;
  rclcpp::Service<IK>::SharedPtr ik_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_;
};
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<wheel_human_commander::KinematicsNode>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("wheel_human_kinematics"), "Kinematics startup failed: %s",
                 e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
