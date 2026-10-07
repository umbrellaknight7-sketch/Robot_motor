#include "wheel_human_commander/ownership_manager.hpp"
#include "wheel_human_commander/robot_state_cache.hpp"
#include "wheel_human_commander/goal_validator.hpp"
#include "wheel_human_commander/execution_client.hpp"

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <erob_driver/config.hpp>
#include <wheel_human_interfaces/action/move_to_pose.hpp>
#include <wheel_human_interfaces/msg/commander_status.hpp>
#include <wheel_human_interfaces/srv/acquire_control.hpp>
#include <wheel_human_interfaces/srv/renew_control.hpp>
#include <wheel_human_interfaces/srv/release_control.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <map>
#include <set>
#include <cmath>
#include <chrono>
#include <functional>

namespace wheel_human_commander {
namespace {
double steady() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

using Acquire = wheel_human_interfaces::srv::AcquireControl;
using Renew = wheel_human_interfaces::srv::RenewControl;
using Release = wheel_human_interfaces::srv::ReleaseControl;
using Move = wheel_human_interfaces::action::MoveJoints;
using Pose = wheel_human_interfaces::action::MoveToPose;
using Goal = rclcpp_action::ServerGoalHandle<Move>;
}

class Commander final : public rclcpp::Node {
public:
  Commander() : Node("wheel_human_commander") {
    const auto file = declare_parameter<std::string>("config_file", "");
    config_ = erob::load_config(file);
    if (!config_.csp_feedforward)
      throw std::runtime_error("Commander requires the CSP feedforward controller/profile");
    for (const auto &d : config_.drives)
      names_.push_back(d.joint);

    // Body ownership and wrist availability are tracked separately.
    owner_ = std::make_unique<OwnershipManager>(declare_parameter<double>("lease_seconds", 2));
    state_ = std::make_unique<RobotStateCache>(names_);
    wrists_enabled_ = declare_parameter<bool>("wrists_enabled", false);
    wrist_state_ = std::make_unique<RobotStateCache>(wrist_names_);
    wrist_stop_timeout_ = declare_parameter<double>("wrist_stop_timeout", 3.);
    if (!std::isfinite(wrist_stop_timeout_) || wrist_stop_timeout_ <= 0)
      throw std::runtime_error("Invalid wrist stop timeout");
    state_timeout_ = declare_parameter<double>("state_timeout", .5);
    handover_timeout_ = declare_parameter<double>("handover_timeout", 15);
    if (!std::isfinite(state_timeout_) || state_timeout_ <= 0 ||
        !std::isfinite(handover_timeout_) || handover_timeout_ <= 0)
      throw std::runtime_error("Invalid commander timeout");
    const auto controller = declare_parameter<std::string>("controller", "/csp_controller");
    hardware_ = declare_parameter<std::string>("hardware", "ErobSystem");
    execution_ = std::make_unique<ExecutionClient>(this, controller);
    if (wrists_enabled_) {
      wrist_execution_ = std::make_unique<ExecutionClient>(this, "/wheel_human_wrist_adapter");
      wrist_heartbeat_ =
          create_publisher<std_msgs::msg::Empty>("/wheel_human_wrist_adapter/heartbeat", 1);
      wrist_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
          "/wheel_human_wrist_adapter/status", 10,
          [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) {
            if (!fresh_stamp(m->header.stamp))
              return;
            for (const auto &status : m->status) {
              if (std::find(wrist_names_.begin(), wrist_names_.end(), status.name) ==
                  wrist_names_.end())
                continue;
              auto v = fields(status);
              try {
                wrist_ready_[status.name] = v.at("ready") == "true";
                wrist_phase_[status.name] = v.at("phase");
                wrist_generation_[status.name] = std::stoull(v.at("generation"));
                wrist_received_[status.name] = steady();
                wrist_fault_[status.name] = status.level == 2;
              } catch (const std::exception &) {
                wrist_received_[status.name] = 0;
              }
            }
          });
    }

    // Only fresh feedback may update the measured-state caches.
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
          if (fresh_stamp(m->header.stamp)) {
            state_->update(*m, steady());
            wrist_state_->update(*m, steady());
          } else {
            state_->invalidate(m->name);
            wrist_state_->invalidate(m->name);
          }
        });
    hardware_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) { hardware_status(*m); });
    controller_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        controller + "/status", 10,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) {
          if (m->status.size() != 1 || !fresh_stamp(m->header.stamp)) {
            controller_received_ = 0;
            return;
          }
          auto values = fields(m->status.front());
          try {
            controller_phase_ = m->status[0].message;
            controller_generation_ = std::stoull(values.at("generation"));
            controller_neutral_ = values.at("neutral") == "true";
            controller_age_ = std::stod(values.at("state_age"));
            controller_received_ = steady();
          } catch (const std::exception &) {
            controller_received_ = 0;
          }
        });
    heartbeat_ = create_publisher<std_msgs::msg::Empty>(controller + "/heartbeat", 1);
    heartbeat_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
      heartbeat_->publish(std_msgs::msg::Empty{});
      if (wrist_heartbeat_ && !stopping_ && !wrist_stopping_)
        wrist_heartbeat_->publish(std_msgs::msg::Empty{});
    });
    status_pub_ = create_publisher<wheel_human_interfaces::msg::CommanderStatus>("~/status", 10);

    // Ownership changes complete after the previous motion has stopped.
    acquire_ = create_service<Acquire>(
        "~/acquire_control",
        [this](std::shared_ptr<rmw_request_id_t> header,
               std::shared_ptr<Acquire::Request> request) { acquire(header, request); });
    release_ = create_service<Release>(
        "~/release_control", [this](std::shared_ptr<rmw_request_id_t> header,
                                    std::shared_ptr<Release::Request> request) {
          if (!owner_->valid(request->token, steady()) || stopping_) {
            Release::Response r;
            r.message = "Not owner or stopping";
            release_->send_response(*header, r);
            return;
          }
          transition_ = [this, header](bool ok) {
            Release::Response r;
            r.success = ok;
            r.message = message_;
            release_->send_response(*header, r);
          };
          owner_->revoke();
          begin_stop("Ownership released");
        });
    renew_ =
        create_service<Renew>("~/renew_control", [this](std::shared_ptr<Renew::Request> request,
                                                        std::shared_ptr<Renew::Response> response) {
          response->success = owner_->renew(request->token, steady());
          response->lease_remaining = owner_->remaining(steady());
          response->message = response->success ? "Renewed" : "Invalid or expired owner";
        });
    reset_ = create_service<std_srvs::srv::Trigger>(
        "~/reset_fault", [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
          r->success = !stopping_ && owner_->owner().empty() && !goal_ && !pending_ && healthy() &&
                       stationary() && controller_phase_ == "holding" && controller_neutral_;
          if (r->success) {
            fault_ = false;
            message_ = "Fault acknowledged; acquire ownership explicitly";
          }
          r->message =
              r->success ? message_ : "Requires healthy stationary hardware and neutral controller";
        });

    // Validate before reserving a goal; dispatch rechecks ownership and readiness.
    move_ = rclcpp_action::create_server<Move>(
        this, "~/move_joints",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Move::Goal> g) {
          if (!owner_->valid(g->token, steady()) || !ready() || pending_ || goal_)
            return rclcpp_action::GoalResponse::REJECT;
          goal_is_wrist_ =
              !g->joint_names.empty() && std::find(wrist_names_.begin(), wrist_names_.end(),
                                                   g->joint_names.front()) != wrist_names_.end();
          prepared_ = goal_is_wrist_ ? validate_wrist(*g) : validate_goal(*g, config_, *state_);
          // Unsupported computation is an accepted action with an explicit result,
          // while malformed or unsafe motion requests are rejected before dispatch.
          if (!prepared_ && prepared_.code != Code::NotImplemented) {
            message_ = prepared_.message;
            return rclcpp_action::GoalResponse::REJECT;
          }
          pending_ = true;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<Goal> g) {
          if (g != goal_)
            return rclcpp_action::CancelResponse::REJECT;
          begin_stop("Goal canceled");
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<Goal> g) { execute(g); });

    // Cartesian queries do not enable Cartesian motion execution.
    pose_ = rclcpp_action::create_server<Pose>(
        this, "~/move_to_pose",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Pose::Goal> g) {
          return owner_->valid(g->token, steady()) && !goal_ && !pending_ && !stopping_
                     ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                     : rclcpp_action::GoalResponse::REJECT;
        },
        [](std::shared_ptr<rclcpp_action::ServerGoalHandle<Pose>>) {
          return rclcpp_action::CancelResponse::REJECT;
        },
        [](std::shared_ptr<rclcpp_action::ServerGoalHandle<Pose>> g) {
          auto result = std::make_shared<Pose::Result>();
          result->error_code = int(Code::NotImplemented);
          result->message =
              "Cartesian execution is not implemented; use the standalone FK/IK query services";
          g->abort(result);
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { tick(); });
  }

private:
  bool fresh_stamp(const builtin_interfaces::msg::Time &stamp) const {
    if (stamp.sec == 0 && stamp.nanosec == 0)
      return false;
    const double age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
    return std::isfinite(age) && age >= -.05 && age <= state_timeout_;
  }

  static std::map<std::string, std::string>
  fields(const diagnostic_msgs::msg::DiagnosticStatus &s) {
    std::map<std::string, std::string> r;
    for (const auto &kv : s.values)
      r[kv.key] = kv.value;
    return r;
  }

  void hardware_status(const diagnostic_msgs::msg::DiagnosticArray &m) {
    if (!fresh_stamp(m.header.stamp)) {
      hardware_good_ = false;
      return;
    }
    std::set<std::string> seen;
    bool good = true;
    for (const auto &s : m.status) {
      if (s.name.rfind(hardware_ + "/", 0) != 0)
        continue;
      auto name = s.name.substr(hardware_.size() + 1);
      if (std::find(names_.begin(), names_.end(), name) == names_.end())
        continue;
      auto v = fields(s);
      good = good && v["phase_name"] == "active" && v["feedback_valid"] == "1" &&
             v["error_code"] == "0";
      if (!seen.insert(name).second)
        good = false;
    }
    if (seen.size() == names_.size()) {
      hardware_good_ = good;
      hardware_received_ = steady();
    }
  }

  bool healthy() const {
    const double now = steady();
    return state_->fresh(now, state_timeout_) && hardware_good_ &&
           now - hardware_received_ <= state_timeout_ && std::isfinite(controller_age_) &&
           controller_age_ >= 0 && now - controller_received_ + controller_age_ <= state_timeout_ &&
           controller_phase_ != "inactive" && controller_phase_ != "fault" && execution_->ready();
  }

  bool stationary() const {
    if (state_->velocities().size() != config_.drives.size())
      return false;
    for (size_t i = 0; i < config_.drives.size(); ++i)
      if (std::abs(state_->velocities()[i]) > config_.drives[i].standstill_velocity)
        return false;
    return true;
  }

  bool ready() const {
    return !fault_ && !stopping_ && !wrist_stopping_ && healthy() && stationary() &&
           controller_phase_ == "holding";
  }

  // Wrist goals use the adapter and cannot mix body joints or torque feedforward.
  ValidatedGoal validate_wrist(const Move::Goal &g) {
    ValidatedGoal r;
    if (!wrists_enabled_ || !wrist_execution_->ready() || wrist_execution_->pending()) {
      r.message = "Wrist adapter unavailable";
      return r;
    }
    if (g.gravity_compensation || !g.torque_joint_names.empty() || !g.torque_offsets.empty()) {
      r.code = Code::NotImplemented;
      r.message = "Wrist gravity/torque feedforward unsupported";
      return r;
    }
    r.duration = g.duration.sec + g.duration.nanosec * 1e-9;
    if (!std::isfinite(r.duration) || r.duration <= 0 || r.duration > 3600 ||
        g.joint_names.size() != g.positions.size()) {
      r.message = "Invalid wrist goal";
      return r;
    }
    std::set<std::string> seen;
    for (size_t i = 0; i < g.joint_names.size(); ++i) {
      const auto &n = g.joint_names[i];
      if (std::find(wrist_names_.begin(), wrist_names_.end(), n) == wrist_names_.end() ||
          !seen.insert(n).second || !std::isfinite(g.positions[i])) {
        r.message = "Mixed body/wrist, unknown or duplicate joint";
        return r;
      }
      if (!wrist_ready_[n] || steady() - wrist_received_[n] > state_timeout_ ||
          !wrist_state_->fresh(n, steady(), state_timeout_)) {
        r.message = "Requested wrist unavailable; body remains available";
        return r;
      }
    }
    r.position = g.positions;
    r.code = Code::Success;
    return r;
  }

  void stop_wrist(Code code, const std::string &reason) {
    if (wrist_stopping_ || !goal_ || !goal_is_wrist_)
      return;
    wrist_stopping_ = true;
    wrist_stop_started_ = steady();
    wrist_stop_generation_ = 0;
    wrist_stop_code_ = code;
    wrist_message_ = reason;
    wrist_execution_->stop([this](bool ok, uint64_t generation) {
      if (wrist_stopping_ && ok)
        wrist_stop_generation_ = generation;
    });
  }

  void acquire(std::shared_ptr<rmw_request_id_t> header,
               std::shared_ptr<Acquire::Request> request) {
    auto answer = [this, header, request](bool ok) {
      Acquire::Response r;
      r.success = ok;
      if (ok) {
        r.token = owner_->acquire(request->source_id, steady());
        r.success = !r.token.empty();
      }
      r.message = r.success ? "Ownership granted" : message_;
      acquire_->send_response(*header, r);
    };
    if (request->source_id.empty() || request->source_id.size() > 128 || stopping_ || transition_ ||
        fault_) {
      message_ = "Invalid source, stopping, or fault";
      answer(false);
      return;
    }
    if (owner_->owner().empty()) {
      if (!ready() || !controller_neutral_) {
        message_ = "Controller must be active, stationary and neutral";
        answer(false);
      } else
        answer(true);
      return;
    }
    if (!request->takeover) {
      message_ = "Robot already owned; explicit takeover required";
      answer(false);
      return;
    }

    // Revoke the old token before waiting for the takeover stop acknowledgement.
    transition_ = answer;
    owner_->revoke();
    begin_stop("Explicit ownership takeover");
  }

  void execute(std::shared_ptr<Goal> g) {
    pending_ = false;
    goal_ = g;
    if (!prepared_) {
      finish(g, prepared_.code, prepared_.message);
      return;
    }
    if (!owner_->valid(g->get_goal()->token, steady()) || !ready()) {
      finish(g, Code::NotReady, "Ownership or readiness changed");
      return;
    }
    if (goal_is_wrist_ && !validate_wrist(*g->get_goal())) {
      finish(g, Code::NotReady, "Wrist readiness changed");
      return;
    }
    const auto trajectory =
        discrete_endpoint(goal_is_wrist_ ? g->get_goal()->joint_names : names_, prepared_.position,
                          prepared_.torque, prepared_.duration);
    message_ = "Executing discrete goal";
    (goal_is_wrist_ ? wrist_execution_ : execution_)
        ->execute(
            trajectory,
            [this, g](const ExecutionClient::Handle::WrappedResult &r) {
              if (g != goal_)
                return;
              if (stopping_ || wrist_stopping_)
                return; // Finalize only after a fresh stationary/neutral acknowledgement.
              if (r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result &&
                  r.result->error_code == 0)
                finish(g, Code::Success, "Goal reached");
              else if (goal_is_wrist_) {
                wrist_message_ = "Wrist execution failed; body ownership retained";
                finish(g, Code::ExecutionFailed, wrist_message_);
              } else {
                owner_->revoke();
                begin_stop("Execution failed or watchdog stopped the controller",
                           Code::ExecutionFailed);
              }
            },
            [this, g](bool accepted) {
              if (!accepted && g == goal_) {
                if (stopping_ || wrist_stopping_)
                  return;
                finish(g, Code::ExecutionFailed, "Controller rejected the goal");
              }
            },
            [this, g](const ExecutionClient::Action::Feedback &f) {
              if (g != goal_)
                return;
              auto out = std::make_shared<Move::Feedback>();
              out->state = stopping_ ? "stopping" : "executing";
              out->joint_names = f.joint_names;
              out->actual_positions = f.actual.positions;
              out->progress = std::clamp(
                  (f.desired.time_from_start.sec + f.desired.time_from_start.nanosec * 1e-9) /
                      prepared_.duration,
                  0., 1.);
              g->publish_feedback(out);
            });
  }

  void finish(std::shared_ptr<Goal> g, Code code, const std::string &text) {
    auto r = std::make_shared<Move::Result>();
    r->error_code = int(code);
    r->message = text;
    if (g->is_canceling() && code == Code::Canceled)
      g->canceled(r);
    else if (code == Code::Success)
      g->succeed(r);
    else
      g->abort(r);
    if (g == goal_)
      goal_.reset();
    message_ = text;
  }

  void begin_stop(const std::string &reason, Code result = Code::Canceled) {
    if (stopping_)
      return;
    stop_wrist(result, reason);
    stopping_ = true;
    stop_result_ = result;
    stop_started_ = steady();
    stop_generation_ = 0;
    message_ = reason;
    execution_->stop([this](bool ok, uint64_t generation) {
      if (!stopping_)
        return;
      if (ok)
        stop_generation_ = generation;
      else
        fail_stop("Controller stop request failed");
    });
  }

  void fail_stop(const std::string &why) {
    fault_ = true;
    stopping_ = false;
    wrist_stopping_ = false;
    owner_->revoke();
    message_ = why;
    if (goal_)
      finish(goal_, Code::ExecutionFailed, why);
    auto callback = std::move(transition_);
    transition_ = {};
    if (callback)
      callback(false);
  }

  // Monitor lease expiry and fresh stop acknowledgements before handing over control.
  void tick() {
    const double now = steady();
    if (owner_->expired(now)) {
      owner_->revoke();
      begin_stop("Ownership lease expired");
    }
    if (!stopping_ && !owner_->owner().empty() && !healthy()) {
      owner_->revoke();
      begin_stop("Hardware or state feedback unavailable", Code::ExecutionFailed);
    }
    if (goal_ && goal_is_wrist_ && !wrist_stopping_) {
      for (const auto &n : goal_->get_goal()->joint_names)
        if (now - wrist_received_[n] > state_timeout_ ||
            !wrist_state_->fresh(n, now, state_timeout_)) {
          stop_wrist(Code::ExecutionFailed, "Wrist feedback timeout; isolated from body");
          break;
        }
    }

    // Confirm the requested wrists independently of the body stop.
    if (wrist_stopping_) {
      bool held = wrist_stop_generation_ && !wrist_execution_->pending();
      bool failed = false;
      for (const auto &n : goal_->get_goal()->joint_names) {
        held = held && wrist_received_[n] > wrist_stop_started_ &&
               now - wrist_received_[n] <= state_timeout_ &&
               wrist_generation_[n] >= wrist_stop_generation_ && wrist_phase_[n] == "holding";
        failed = failed || wrist_fault_[n];
      }
      if (held || now - wrist_stop_started_ > wrist_stop_timeout_) {
        wrist_stopping_ = false;
        if (!held || failed) {
          wrist_stop_code_ = Code::ExecutionFailed;
          wrist_message_ = "Wrist stop unavailable; isolated, explicit wrist reset required";
        }
        if (goal_)
          finish(goal_, wrist_stop_code_, wrist_message_);
      }
    }
    if (stopping_) {
      if (!wrist_stopping_ && stop_generation_ && controller_generation_ >= stop_generation_ &&
          controller_received_ > stop_started_ && controller_phase_ == "holding" &&
          controller_neutral_ && healthy() && stationary() && !execution_->pending()) {
        stopping_ = false;
        if (goal_)
          finish(goal_, stop_result_, message_);
        auto callback = std::move(transition_);
        transition_ = {};
        if (callback)
          callback(true);
      } else if (now - stop_started_ > handover_timeout_)
        fail_stop("Stop not confirmed; ownership handover blocked");
    }

    wheel_human_interfaces::msg::CommanderStatus s;
    s.header.stamp = this->now();
    s.owner = owner_->owner();
    s.ready = ready() && !goal_ && !pending_;
    s.state =
        fault_
            ? "fault"
            : (stopping_ ? "stopping"
                         : (goal_ || pending_ ? "executing" : (s.ready ? "ready" : "not_ready")));
    s.active_goal_id = goal_ ? rclcpp_action::to_string(goal_->get_goal_id()) : "";
    s.wrist_enabled = wrists_enabled_;
    s.wrist_message = wrist_message_;
    for (const auto &n : wrist_names_) {
      s.wrist_joint_names.push_back(n);
      s.wrist_ready.push_back(wrists_enabled_ && wrist_execution_->ready() &&
                              !wrist_execution_->pending() && wrist_ready_[n] &&
                              now - wrist_received_[n] <= state_timeout_ &&
                              wrist_state_->fresh(n, now, state_timeout_));
    }
    s.lease_remaining = owner_->remaining(now);
    s.joint_goals_supported = true;
    s.feedforward_supported = true;
    s.message = message_;
    status_pub_->publish(s);
  }

  const std::vector<std::string> wrist_names_{"Left_Wrist_Joint", "Right_Wrist_Joint"};
  bool wrists_enabled_ = false, goal_is_wrist_ = false, wrist_stopping_ = false;
  double wrist_stop_timeout_ = 3, wrist_stop_started_ = 0;
  uint64_t wrist_stop_generation_ = 0;
  Code wrist_stop_code_ = Code::Canceled;
  std::string wrist_message_;
  std::map<std::string, bool> wrist_ready_, wrist_fault_;
  std::map<std::string, double> wrist_received_;
  std::map<std::string, std::string> wrist_phase_;
  std::map<std::string, uint64_t> wrist_generation_;
  std::unique_ptr<RobotStateCache> wrist_state_;
  std::unique_ptr<ExecutionClient> wrist_execution_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr wrist_heartbeat_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr wrist_sub_;
  erob::BusConfig config_;
  std::vector<std::string> names_;
  std::string hardware_, controller_phase_ = "inactive", message_;
  std::unique_ptr<OwnershipManager> owner_;
  std::unique_ptr<RobotStateCache> state_;
  std::unique_ptr<ExecutionClient> execution_;
  ValidatedGoal prepared_;
  Code stop_result_ = Code::Canceled;
  bool pending_ = false, stopping_ = false, fault_ = false, hardware_good_ = false,
       controller_neutral_ = false;
  double state_timeout_ = .5, handover_timeout_ = 15, hardware_received_ = 0,
         controller_received_ = 0, controller_age_ = 0, stop_started_ = 0;
  uint64_t controller_generation_ = 0, stop_generation_ = 0;
  std::function<void(bool)> transition_;
  std::shared_ptr<Goal> goal_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr hardware_sub_,
      controller_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_;
  rclcpp::Publisher<wheel_human_interfaces::msg::CommanderStatus>::SharedPtr status_pub_;
  rclcpp::Service<Acquire>::SharedPtr acquire_;
  rclcpp::Service<Renew>::SharedPtr renew_;
  rclcpp::Service<Release>::SharedPtr release_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;
  rclcpp_action::Server<Move>::SharedPtr move_;
  rclcpp_action::Server<Pose>::SharedPtr pose_;
  rclcpp::TimerBase::SharedPtr timer_, heartbeat_timer_;
};
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<wheel_human_commander::Commander>());
  } catch (const std::exception &e) {
    fprintf(stderr, "Commander: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
