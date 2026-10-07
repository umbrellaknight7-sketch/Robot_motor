#include "kinematics_impl.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <stdexcept>

namespace wheel_human_commander {
void KinematicsModel::Impl::evaluate(const Eigen::VectorXd &q) {
  pinocchio::framesForwardKinematics(model, *data, q);
}

KinematicsResult KinematicsModel::forward(const sensor_msgs::msg::JointState &input,
                                          const std::string &frame) {
  KinematicsResult result;
  try {
    auto q = impl_->configuration(input);
    auto id = impl_->frame(frame);
    impl_->evaluate(q);
    result.pose = impl_->pose(id);
    result.code = KinematicsCode::Success;
    result.message = "FK calculated";
  } catch (const std::invalid_argument &e) {
    result.message = e.what();
  } catch (const std::exception &e) {
    result.code = KinematicsCode::InternalError;
    result.message = e.what();
  }
  return result;
}

JacobianResult KinematicsModel::jacobian(const sensor_msgs::msg::JointState &input,
                                         const std::string &frame) {
  JacobianResult result;
  try {
    auto q = impl_->configuration(input);
    auto id = impl_->frame(frame);
    pinocchio::Data::Matrix6x J(6, impl_->model.nv);
    J.setZero();

    // Express the Jacobian at the target frame origin with axes aligned to the world.
    pinocchio::computeFrameJacobian(impl_->model, *impl_->data, q, id,
                                    pinocchio::LOCAL_WORLD_ALIGNED, J);
    impl_->evaluate(q);
    result.pose = impl_->pose(id);
    result.joint_names = impl_->names;
    for (int row = 0; row < 6; ++row)
      for (const auto &name : impl_->names)
        result.rows[row].push_back(J(row, impl_->v_indices.at(name)));
    result.code = KinematicsCode::Success;
    result.message = "Frame Jacobian calculated";
  } catch (const std::invalid_argument &e) {
    result.message = e.what();
  } catch (const std::exception &e) {
    result.code = KinematicsCode::InternalError;
    result.message = e.what();
  }
  return result;
}
}
