#ifndef PAYLOAD_POINTMASS_MULTIPLE_QUADROTOR__NMPC_PLANNER_POINTMASS_MULTIPLE_H_
#define PAYLOAD_POINTMASS_MULTIPLE_QUADROTOR__NMPC_PLANNER_POINTMASS_MULTIPLE_H_

#include "payload_pointmass_multiple_quadrotor/wrapper_pointmass_multiple.h"

namespace payload_pointmass_multiple_quadrotor {

class NMPCControlPointMassMultiple {
public:
  NMPCControlPointMassMultiple();

  void setState(
      const Eigen::Matrix<double, kStateSizePointMassMultiple, 1> &state,
      double stamp);
  void setReferenceStates(
      const Eigen::Matrix<double, kStateSizePointMassMultiple,
                          kSamplesPointMassMultiple> &reference_states);
  void setReferenceInputs(
      const Eigen::Matrix<double, kInputSizePointMassMultiple,
                          kSamplesPointMassMultiple> &reference_inputs);

  Eigen::Matrix<double, kStateSizePointMassMultiple, 1> getPredictedState();
  Eigen::Matrix<double, kInputSizePointMassMultiple, 1> getPredictedInput();
  Eigen::Matrix<double, kStateSizePointMassMultiple, kSamplesPointMassMultiple>
  getPredictedStates();
  Eigen::Matrix<double, kInputSizePointMassMultiple, kSamplesPointMassMultiple>
  getPredictedInputs();
  Eigen::Matrix<double, kStateSizePointMassMultiple, kSamplesPointMassMultiple>
  getReferenceStates();
  Eigen::Matrix<double, kInputSizePointMassMultiple, kSamplesPointMassMultiple>
  getReferenceInputs();
  int run();
  double getStampState();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  bool solve_from_scratch_;
  double stamp_current_state_{0.0};
  Eigen::Matrix<double, kStateSizePointMassMultiple, 1> current_state_;
  Eigen::Matrix<double, kStateSizePointMassMultiple, kSamplesPointMassMultiple>
      reference_states_;
  Eigen::Matrix<double, kInputSizePointMassMultiple, kSamplesPointMassMultiple>
      reference_inputs_;
  Eigen::Matrix<double, kStateSizePointMassMultiple, kSamplesPointMassMultiple>
      predicted_states_;
  Eigen::Matrix<double, kInputSizePointMassMultiple, kSamplesPointMassMultiple>
      predicted_inputs_;

  NMPCWrapperPointMassMultiple wrapper_;
};

} // namespace payload_pointmass_multiple_quadrotor

#endif
