#include "wheel_human_commander/execution_client.hpp"

namespace wheel_human_commander {
ExecutionClient::ExecutionClient(rclcpp::Node *n, const std::string &name) {
  action_ = rclcpp_action::create_client<Action>(n, name + "/follow_joint_trajectory");
  stop_ = n->create_client<std_srvs::srv::Trigger>(name + "/stop");
}

bool ExecutionClient::ready() const {
  return action_->action_server_is_ready() && stop_->service_is_ready();
}

void ExecutionClient::execute(const trajectory_msgs::msg::JointTrajectory &t,
                              std::function<void(const Handle::WrappedResult &)> result,
                              std::function<void(bool)> accepted,
                              std::function<void(const Action::Feedback &)> feedback) {
  pending_ = true;
  handle_.reset();
  Action::Goal g;
  g.trajectory = t;
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [this, accepted](Handle::SharedPtr h) {
    pending_ = false;
    handle_ = h;
    accepted(bool(h));

    // A stop requested during goal acceptance must follow the accepted goal.
    auto stopping = std::move(deferred_stop_);
    deferred_stop_ = {};
    if (stopping)
      stop(std::move(stopping));
  };
  options.result_callback = [this, result](const Handle::WrappedResult &r) {
    handle_.reset();
    result(r);
  };
  options.feedback_callback =
      [feedback](Handle::SharedPtr, std::shared_ptr<const Action::Feedback> f) { feedback(*f); };
  action_->async_send_goal(g, options);
}

// Return the controller stop generation so the caller can confirm fresh completion.
void ExecutionClient::stop(std::function<void(bool, uint64_t)> done) {
  if (pending_) {
    deferred_stop_ = std::move(done);
    return;
  }
  if (!stop_->service_is_ready()) {
    done(false, 0);
    return;
  }
  stop_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
                            [done](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
                              try {
                                auto r = f.get();
                                done(r->success, r->success ? std::stoull(r->message) : 0);
                              } catch (const std::exception &) {
                                done(false, 0);
                              }
                            });
}
}
