#include "payload_pointmass_multiple_quadrotor/nmpc_planner_pointmass_multiple.h"

#include <algorithm>
#include <iostream>

namespace payload_pointmass_multiple_quadrotor {

NMPCControlPointMassMultiple::NMPCControlPointMassMultiple()
    : solve_from_scratch_(true),
      current_state_(
          Eigen::Matrix<double, kStateSizePointMassMultiple, 1>::Zero()),
      reference_states_(Eigen::Matrix<double, kStateSizePointMassMultiple,
                                      kSamplesPointMassMultiple>::Zero()),
      reference_inputs_(Eigen::Matrix<double, kInputSizePointMassMultiple,
                                      kSamplesPointMassMultiple>::Zero()),
      predicted_states_(Eigen::Matrix<double, kStateSizePointMassMultiple,
                                      kSamplesPointMassMultiple>::Zero()),
      predicted_inputs_(Eigen::Matrix<double, kInputSizePointMassMultiple,
                                      kSamplesPointMassMultiple>::Zero()) {
  current_state_(8) = -1.0;
  current_state_(11) = -1.0;
  current_state_(14) = -1.0;
  reference_states_.row(8).setConstant(-1.0);
  reference_states_.row(11).setConstant(-1.0);
  reference_states_.row(14).setConstant(-1.0);
  predicted_states_.row(8).setConstant(-1.0);
  predicted_states_.row(11).setConstant(-1.0);
  predicted_states_.row(14).setConstant(-1.0);
}

void NMPCControlPointMassMultiple::setState(
    const Eigen::Matrix<double, kStateSizePointMassMultiple, 1> &state,
    double stamp) {
  current_state_ = state;
  stamp_current_state_ = stamp;
}

void NMPCControlPointMassMultiple::setReferenceStates(
    const Eigen::Matrix<double, kStateSizePointMassMultiple,
                        kSamplesPointMassMultiple> &reference_states) {
  reference_states_ = reference_states;
}

void NMPCControlPointMassMultiple::setReferenceInputs(
    const Eigen::Matrix<double, kInputSizePointMassMultiple,
                        kSamplesPointMassMultiple> &reference_inputs) {
  reference_inputs_ = reference_inputs;
}

double NMPCControlPointMassMultiple::getStampState() {
  return stamp_current_state_;
}

Eigen::Matrix<double, kStateSizePointMassMultiple, 1>
NMPCControlPointMassMultiple::getPredictedState() {
  return predicted_states_.col(std::min(1, kSamplesPointMassMultiple - 1));
}

Eigen::Matrix<double, kInputSizePointMassMultiple, 1>
NMPCControlPointMassMultiple::getPredictedInput() {
  return predicted_inputs_.col(0);
}

Eigen::Matrix<double, kStateSizePointMassMultiple, kSamplesPointMassMultiple>
NMPCControlPointMassMultiple::getPredictedStates() {
  return predicted_states_;
}

Eigen::Matrix<double, kInputSizePointMassMultiple, kSamplesPointMassMultiple>
NMPCControlPointMassMultiple::getPredictedInputs() {
  return predicted_inputs_;
}

Eigen::Matrix<double, kStateSizePointMassMultiple, kSamplesPointMassMultiple>
NMPCControlPointMassMultiple::getReferenceStates() {
  return reference_states_;
}

Eigen::Matrix<double, kInputSizePointMassMultiple, kSamplesPointMassMultiple>
NMPCControlPointMassMultiple::getReferenceInputs() {
  return reference_inputs_;
}

int NMPCControlPointMassMultiple::run() {
  wrapper_.setTrajectory(reference_states_, reference_inputs_);
  if (solve_from_scratch_) {
    std::cout
        << "Solving point-mass multiple NMPC with hover as initial guess.\n";
    wrapper_.prepare(current_state_);
    solve_from_scratch_ = false;
  }

  const int acados_status = wrapper_.update(current_state_);
  wrapper_.getStates(predicted_states_);
  wrapper_.getInputs(predicted_inputs_);
  return acados_status;
}

} // namespace payload_pointmass_multiple_quadrotor
