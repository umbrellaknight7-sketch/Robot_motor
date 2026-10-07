#include "kinematics_impl.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <Eigen/Cholesky>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace wheel_human_commander {
IkResult KinematicsModel::inverse(const sensor_msgs::msg::JointState &seed,
                                  const std::string &group_name,
                                  const geometry_msgs::msg::PoseStamped &target, IkMode mode) {
  IkResult result;
  try {
    auto &s = *impl_;
    Eigen::VectorXd q = s.configuration(seed);
    auto found = s.groups.find(group_name);
    if (found == s.groups.end())
      throw std::invalid_argument("Unknown planning group: " + group_name);
    if (mode != IkMode::PositionOnly && mode != IkMode::FullPose)
      throw std::invalid_argument("Unknown IK target mode");
    if (target.header.frame_id != "base_link")
      throw std::invalid_argument("IK target frame must be base_link");
    const auto &p = target.pose.position;
    Eigen::Vector3d position(p.x, p.y, p.z);
    if (!position.allFinite())
      throw std::invalid_argument("Target position must be finite");
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    if (mode == IkMode::FullPose) {
      const auto &o = target.pose.orientation;
      Eigen::Quaterniond quaternion(o.w, o.x, o.y, o.z);
      if (!quaternion.coeffs().allFinite())
        throw std::invalid_argument("Target quaternion must be finite");
      double scale = quaternion.coeffs().cwiseAbs().maxCoeff();
      if (scale == 0)
        throw std::invalid_argument("Full-pose target needs a nonzero quaternion");
      quaternion.coeffs() /= scale;
      quaternion.normalize();
      rotation = quaternion.toRotationMatrix();
    }
    const pinocchio::SE3 desired(rotation, position);
    const auto &group = found->second;
    const auto &options = s.options;

    // Position-only solves use three residuals; full-pose solves add orientation.
    const int dimension = mode == IkMode::PositionOnly ? 3 : 6;
    const int count = static_cast<int>(group.q_indices.size());
    using Clock = std::chrono::steady_clock;
    const auto started = Clock::now();
    const auto expired = [&] {
      return std::chrono::duration<double>(Clock::now() - started).count() >=
             options.timeout_seconds;
    };
    const auto timeout = [&] {
      result.code = KinematicsCode::Timeout;
      result.message = "IK time budget exhausted";
    };
    // Error and Jacobian use the same frame convention. For full pose this is
    // the SE(3) logarithm in the current TCP frame, with the Jlog correction.
    const auto error = [&](const Eigen::VectorXd &state) {
      s.evaluate(state);
      Eigen::VectorXd e(dimension);
      const auto &current = s.data->oMf[group.tip];
      if (mode == IkMode::PositionOnly)
        e = position - current.translation();
      else {
        e = pinocchio::log6(current.actInv(desired)).toVector();
        e.tail<3>() *= options.orientation_scale;
      }
      return e;
    };
    const auto measure = [&] {
      const auto &current = s.data->oMf[group.tip];
      result.pose = s.pose(group.tip);
      result.position_error = (position - current.translation()).norm();
      if (mode == IkMode::FullPose)
        result.orientation_error =
            pinocchio::log3(current.rotation().transpose() * rotation).norm();
      return result.position_error <= options.position_tolerance &&
             (mode == IkMode::PositionOnly ||
              result.orientation_error <= options.orientation_tolerance);
    };
    for (int iteration = 0; iteration <= options.max_iterations; ++iteration) {
      result.iterations = iteration;
      if (expired()) {
        timeout();
        return result;
      }
      Eigen::VectorXd e = error(q);
      bool converged = measure(); // Independent position/orientation FK acceptance.
      if (expired()) {
        timeout();
        return result;
      }
      if (converged) {
        result.solution.name = s.names;
        for (const auto &name : s.names)
          result.solution.position.push_back(q[s.q_indices.at(name)]);
        result.code = KinematicsCode::Success;
        result.message = "IK solution verified by FK";
        return result;
      }
      if (iteration == options.max_iterations) {
        result.message = "IK iteration budget exhausted";
        break;
      }
      if (!e.allFinite()) {
        result.message = "Non-finite IK residual";
        break;
      }
      pinocchio::Data::Matrix6x full(6, s.model.nv);
      full.setZero();
      pinocchio::computeFrameJacobian(
          s.model, *s.data, q, group.tip,
          mode == IkMode::PositionOnly ? pinocchio::LOCAL_WORLD_ALIGNED : pinocchio::LOCAL, full);
      if (mode == IkMode::FullPose) {
        pinocchio::Data::Matrix6 correction;
        pinocchio::Jlog6(s.data->oMf[group.tip].actInv(desired).inverse(), correction);
        full = (correction * full).eval();
        full.bottomRows<3>() *= options.orientation_scale;
      }

      // Select only the planning-group columns; all other joints stay at the seed values.
      Eigen::MatrixXd J(dimension, count);
      for (int i = 0; i < count; ++i)
        J.col(i) = full.col(group.v_indices[i]).head(dimension);
      Eigen::VectorXd step(count);
      std::vector<bool> blocked(count, false);
      // Re-solve after removing any saturated joint whose direction points
      // outside its bounds, allowing the remaining joints to make progress.
      for (int pass = 0; pass <= count; ++pass) {
        Eigen::MatrixXd active = J;
        for (int i = 0; i < count; ++i)
          if (blocked[i])
            active.col(i).setZero();
        Eigen::MatrixXd normal = active * active.transpose();
        normal.diagonal().array() += options.damping * options.damping;
        step = active.transpose() * normal.ldlt().solve(e);
        bool changed = false;
        for (int i = 0; i < count; ++i) {
          int qi = group.q_indices[i];
          if (!blocked[i] && ((q[qi] <= s.model.lowerPositionLimit[qi] + 1e-12 && step[i] < 0) ||
                              (q[qi] >= s.model.upperPositionLimit[qi] - 1e-12 && step[i] > 0))) {
            blocked[i] = true;
            changed = true;
          }
        }
        if (!changed)
          break;
      }
      if (!step.allFinite() || step.cwiseAbs().maxCoeff() < 1e-14) {
        result.message = "IK stalled at a singularity or joint bound";
        break;
      }

      // Limit the proposed joint step before testing residual reduction.
      double maximum = step.cwiseAbs().maxCoeff();
      if (maximum > options.max_joint_step)
        step *= options.max_joint_step / maximum;
      double alpha = 1.;
      for (int i = 0; i < count; ++i) {
        int qi = group.q_indices[i];
        if (step[i] > 0)
          alpha = std::min(alpha, (s.model.upperPositionLimit[qi] - q[qi]) / step[i]);
        else if (step[i] < 0)
          alpha = std::min(alpha, (s.model.lowerPositionLimit[qi] - q[qi]) / step[i]);
      }
      bool accepted = false;
      const double cost = e.squaredNorm();
      for (int trial = 0; trial < 16 && alpha > 1e-12; ++trial, alpha *= .5) {
        if (expired()) {
          timeout();
          return result;
        }
        Eigen::VectorXd candidate = q;
        for (int i = 0; i < count; ++i) {
          int qi = group.q_indices[i];
          candidate[qi] = std::clamp(q[qi] + alpha * step[i], s.model.lowerPositionLimit[qi],
                                     s.model.upperPositionLimit[qi]);
        }
        Eigen::VectorXd next = error(candidate);
        if (next.allFinite() && next.squaredNorm() < cost) {
          q = std::move(candidate);
          accepted = true;
          break;
        }
      }
      if (!accepted) {
        s.evaluate(q);
        measure();
        result.message = "IK line search made no progress";
        break;
      }
    }
    result.code = KinematicsCode::NoConvergence;
  } catch (const std::invalid_argument &e) {
    result.message = e.what();
  } catch (const std::exception &e) {
    result.code = KinematicsCode::InternalError;
    result.message = e.what();
  }
  return result;
}
}
