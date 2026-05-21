#include "payload_pointmass_multiple_quadrotor/wrapper_pointmass_multiple.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <numeric>

namespace payload_pointmass_multiple_quadrotor {

solver_input_pointmass_multiple acados_in_pointmass_multiple;
solver_output_pointmass_multiple acados_out_pointmass_multiple;

NMPCWrapperPointMassMultiple::NMPCWrapperPointMassMultiple() {
  acados_ocp_capsule_ =
      planner_payload_pointmass_multiple_acados_create_capsule();
  const int status =
      planner_payload_pointmass_multiple_acados_create_with_discretization(
          acados_ocp_capsule_, N_POINTMASS_MULTIPLE, new_time_steps_);
  if (status != 0) {
    std::cerr
        << "planner_payload_pointmass_multiple_acados_create() returned status "
        << status << std::endl;
    std::exit(1);
  }

  nlp_config_ =
      planner_payload_pointmass_multiple_acados_get_nlp_config(
          acados_ocp_capsule_);
  nlp_dims_ =
      planner_payload_pointmass_multiple_acados_get_nlp_dims(
          acados_ocp_capsule_);
  nlp_in_ =
      planner_payload_pointmass_multiple_acados_get_nlp_in(
          acados_ocp_capsule_);
  nlp_out_ =
      planner_payload_pointmass_multiple_acados_get_nlp_out(
          acados_ocp_capsule_);
  nlp_solver_ =
      planner_payload_pointmass_multiple_acados_get_nlp_solver(
          acados_ocp_capsule_);
  nlp_opts_ =
      planner_payload_pointmass_multiple_acados_get_nlp_opts(
          acados_ocp_capsule_);

  Eigen::Matrix<double, kStateSizePointMassMultiple, 1> hover_state =
      Eigen::Matrix<double, kStateSizePointMassMultiple, 1>::Zero();
  hover_state(8) = -1.0;
  hover_state(11) = -1.0;
  hover_state(14) = -1.0;
  resetWarmStart(hover_state);
}

void NMPCWrapperPointMassMultiple::resetWarmStart(
    const Eigen::Ref<const Eigen::Matrix<double, kStateSizePointMassMultiple, 1>>
        state) {
  acados_initial_state_ = state;
  acados_states_ = state.replicate(1, kSamplesPointMassMultiple);
  acados_inputs_.setZero();

  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0,
                                "lbx", acados_in_pointmass_multiple.x0);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0,
                                "ubx", acados_in_pointmass_multiple.x0);

  acados_reference_states_.block(0, 0, kStateSizePointMassMultiple,
                                 kSamplesPointMassMultiple) =
      state.replicate(1, kSamplesPointMassMultiple);
  acados_reference_states_.block(kStateSizePointMassMultiple, 0,
                                 kInputSizePointMassMultiple,
                                 kSamplesPointMassMultiple) =
      kHoverInput_.replicate(1, kSamplesPointMassMultiple);
  acados_reference_end_state_.segment(0, kStateSizePointMassMultiple) = state;
  acados_reference_end_state_
      .segment(kStateSizePointMassMultiple, kInputSizePointMassMultiple)
      .setZero();
}

void NMPCWrapperPointMassMultiple::shiftWarmStart(
    const Eigen::Ref<const Eigen::Matrix<double, kStateSizePointMassMultiple, 1>>
        state) {
  if (!acados_is_prepared_) {
    resetWarmStart(state);
    return;
  }

  Eigen::Matrix<double, kStateSizePointMassMultiple,
                kSamplesPointMassMultiple>
      shifted_states = acados_states_;
  Eigen::Matrix<double, kInputSizePointMassMultiple,
                kSamplesPointMassMultiple>
      shifted_inputs = acados_inputs_;
  if (kSamplesPointMassMultiple > 1) {
    shifted_states.leftCols(kSamplesPointMassMultiple - 1) =
        acados_states_.rightCols(kSamplesPointMassMultiple - 1);
    shifted_inputs.leftCols(kSamplesPointMassMultiple - 1) =
        acados_inputs_.rightCols(kSamplesPointMassMultiple - 1);
  }
  shifted_states.col(0) = state;
  shifted_states.col(kSamplesPointMassMultiple - 1) =
      acados_states_.col(kSamplesPointMassMultiple - 1);
  shifted_inputs.col(kSamplesPointMassMultiple - 1) =
      acados_inputs_.col(kSamplesPointMassMultiple - 1);

  acados_states_ = shifted_states;
  acados_inputs_ = shifted_inputs;
  for (int i = 0; i <= N_POINTMASS_MULTIPLE; ++i) {
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x",
                    acados_out_pointmass_multiple.x_out +
                        i * NX_POINTMASS_MULTIPLE);
  }
  for (int i = 0; i < N_POINTMASS_MULTIPLE; ++i) {
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u",
                    acados_out_pointmass_multiple.u_out +
                        i * NU_POINTMASS_MULTIPLE);
  }
}

bool NMPCWrapperPointMassMultiple::prepare(
    const Eigen::Ref<const Eigen::Matrix<double, kStateSizePointMassMultiple, 1>>
        state) {
  resetWarmStart(state);
  for (int i = 0; i <= N_POINTMASS_MULTIPLE; ++i) {
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x",
                    acados_out_pointmass_multiple.x_out +
                        i * NX_POINTMASS_MULTIPLE);
  }
  for (int i = 0; i < N_POINTMASS_MULTIPLE; ++i) {
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u",
                    acados_out_pointmass_multiple.u_out +
                        i * NU_POINTMASS_MULTIPLE);
  }
  acados_is_prepared_ = true;
  return true;
}

int NMPCWrapperPointMassMultiple::update(
    const Eigen::Ref<const Eigen::Matrix<double, kStateSizePointMassMultiple, 1>>
        state) {
  int rti_phase = 0;
  ocp_nlp_solver_opts_set(nlp_config_, nlp_opts_, "rti_phase", &rti_phase);
  shiftWarmStart(state);

  acados_initial_state_ = state;
  ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, 0, "x",
                  acados_in_pointmass_multiple.x0);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0,
                                "lbx", acados_in_pointmass_multiple.x0);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0,
                                "ubx", acados_in_pointmass_multiple.x0);

  int y_indices[kYRefSizePointMassMultiple];
  std::iota(y_indices, y_indices + kYRefSizePointMassMultiple, 0);
  for (int i = 0; i < N_POINTMASS_MULTIPLE; ++i) {
    planner_payload_pointmass_multiple_acados_update_params_sparse(
        acados_ocp_capsule_, i, y_indices,
        acados_in_pointmass_multiple.yref + i * kYRefSizePointMassMultiple,
        kYRefSizePointMassMultiple);
  }
  planner_payload_pointmass_multiple_acados_update_params_sparse(
      acados_ocp_capsule_, N_POINTMASS_MULTIPLE, y_indices,
      acados_in_pointmass_multiple.yref_e, kYRefSizePointMassMultiple);

  const int acados_status =
      planner_payload_pointmass_multiple_acados_solve(acados_ocp_capsule_);

  for (int i = 0; i <= nlp_dims_->N; ++i) {
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x",
                    &acados_out_pointmass_multiple
                         .x_out[i * NX_POINTMASS_MULTIPLE]);
  }
  for (int i = 0; i < nlp_dims_->N; ++i) {
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "u",
                    &acados_out_pointmass_multiple
                         .u_out[i * NU_POINTMASS_MULTIPLE]);
  }
  return acados_status;
}

void NMPCWrapperPointMassMultiple::getStates(
    Eigen::Matrix<double, kStateSizePointMassMultiple, kSamplesPointMassMultiple>
        &return_state) {
  return_state = acados_states_;
}

void NMPCWrapperPointMassMultiple::getInputs(
    Eigen::Matrix<double, kInputSizePointMassMultiple, kSamplesPointMassMultiple>
        &return_input) {
  return_input = acados_inputs_;
}

void NMPCWrapperPointMassMultiple::setTrajectory(
    const Eigen::Ref<const Eigen::Matrix<double, kStateSizePointMassMultiple,
                                         kSamplesPointMassMultiple>> states,
    const Eigen::Ref<const Eigen::Matrix<double, kInputSizePointMassMultiple,
                                         kSamplesPointMassMultiple>> inputs) {
  acados_reference_states_.block(0, 0, kStateSizePointMassMultiple,
                                 kSamplesPointMassMultiple) = states;
  acados_reference_states_.block(kStateSizePointMassMultiple, 0,
                                 kInputSizePointMassMultiple,
                                 kSamplesPointMassMultiple) = inputs;
  acados_reference_end_state_.segment(0, kStateSizePointMassMultiple) =
      states.col(kSamplesPointMassMultiple - 1);
  acados_reference_end_state_
      .segment(kStateSizePointMassMultiple, kInputSizePointMassMultiple)
      .setZero();
}

} // namespace payload_pointmass_multiple_quadrotor
