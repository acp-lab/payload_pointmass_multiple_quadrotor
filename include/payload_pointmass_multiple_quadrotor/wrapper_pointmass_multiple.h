#ifndef PAYLOAD_POINTMASS_MULTIPLE_QUADROTOR__WRAPPER_POINTMASS_MULTIPLE_H_
#define PAYLOAD_POINTMASS_MULTIPLE_QUADROTOR__WRAPPER_POINTMASS_MULTIPLE_H_

#include <vector>

#include <Eigen/Eigen>

#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_planner_payload_pointmass_multiple.h"

#define NX_POINTMASS_MULTIPLE PLANNER_PAYLOAD_POINTMASS_MULTIPLE_NX
#define NZ_POINTMASS_MULTIPLE PLANNER_PAYLOAD_POINTMASS_MULTIPLE_NZ
#define NU_POINTMASS_MULTIPLE PLANNER_PAYLOAD_POINTMASS_MULTIPLE_NU
#define NP_POINTMASS_MULTIPLE PLANNER_PAYLOAD_POINTMASS_MULTIPLE_NP
#define NBU_POINTMASS_MULTIPLE PLANNER_PAYLOAD_POINTMASS_MULTIPLE_NBU
#define NSH_POINTMASS_MULTIPLE PLANNER_PAYLOAD_POINTMASS_MULTIPLE_NSH
const int N_POINTMASS_MULTIPLE = PLANNER_PAYLOAD_POINTMASS_MULTIPLE_N;

namespace payload_pointmass_multiple_quadrotor {

static constexpr int kStateSizePointMassMultiple =
    PLANNER_PAYLOAD_POINTMASS_MULTIPLE_NX;
static constexpr int kSamplesPointMassMultiple =
    PLANNER_PAYLOAD_POINTMASS_MULTIPLE_N;
static constexpr int kInputSizePointMassMultiple =
    PLANNER_PAYLOAD_POINTMASS_MULTIPLE_NU;
static constexpr int kYRefSizePointMassMultiple =
    PLANNER_PAYLOAD_POINTMASS_MULTIPLE_NX +
    PLANNER_PAYLOAD_POINTMASS_MULTIPLE_NU;

struct solver_output_pointmass_multiple {
  double status, KKT_res, cpu_time;
  double u0[NU_POINTMASS_MULTIPLE];
  double u1[NU_POINTMASS_MULTIPLE];
  double x1[NX_POINTMASS_MULTIPLE];
  double x2[NX_POINTMASS_MULTIPLE];
  double x4[NX_POINTMASS_MULTIPLE];
  double xi[NU_POINTMASS_MULTIPLE];
  double ui[NU_POINTMASS_MULTIPLE];
  double u_out[NU_POINTMASS_MULTIPLE * N_POINTMASS_MULTIPLE];
  double x_out[NX_POINTMASS_MULTIPLE * (N_POINTMASS_MULTIPLE + 1)];
};

struct solver_input_pointmass_multiple {
  double x0[NX_POINTMASS_MULTIPLE];
  double x[NX_POINTMASS_MULTIPLE * N_POINTMASS_MULTIPLE];
  double u[NU_POINTMASS_MULTIPLE * N_POINTMASS_MULTIPLE];
  double yref[(NX_POINTMASS_MULTIPLE + NU_POINTMASS_MULTIPLE) *
              N_POINTMASS_MULTIPLE];
  double yref_e[(NX_POINTMASS_MULTIPLE + NU_POINTMASS_MULTIPLE)];
};

extern solver_input_pointmass_multiple acados_in_pointmass_multiple;
extern solver_output_pointmass_multiple acados_out_pointmass_multiple;

class NMPCWrapperPointMassMultiple {
public:
  NMPCWrapperPointMassMultiple();

  bool prepare(const Eigen::Ref<
               const Eigen::Matrix<double, kStateSizePointMassMultiple, 1>>
                   state);
  int update(const Eigen::Ref<
             const Eigen::Matrix<double, kStateSizePointMassMultiple, 1>>
                 state);

  void getStates(Eigen::Matrix<double, kStateSizePointMassMultiple,
                                kSamplesPointMassMultiple> &return_state);
  void getInputs(Eigen::Matrix<double, kInputSizePointMassMultiple,
                                kSamplesPointMassMultiple> &return_input);

  void setTrajectory(
      const Eigen::Ref<const Eigen::Matrix<double, kStateSizePointMassMultiple,
                                           kSamplesPointMassMultiple>> states,
      const Eigen::Ref<const Eigen::Matrix<double, kInputSizePointMassMultiple,
                                           kSamplesPointMassMultiple>> inputs);

private:
  void resetWarmStart(const Eigen::Ref<
                      const Eigen::Matrix<double, kStateSizePointMassMultiple,
                                          1>> state);
  void shiftWarmStart(const Eigen::Ref<
                      const Eigen::Matrix<double, kStateSizePointMassMultiple,
                                          1>> state);

  planner_payload_pointmass_multiple_solver_capsule *acados_ocp_capsule_{
      nullptr};
  ocp_nlp_in *nlp_in_{nullptr};
  ocp_nlp_out *nlp_out_{nullptr};
  ocp_nlp_solver *nlp_solver_{nullptr};
  void *nlp_opts_{nullptr};
  ocp_nlp_config *nlp_config_{nullptr};
  ocp_nlp_dims *nlp_dims_{nullptr};
  double *new_time_steps_{nullptr};
  bool acados_is_prepared_{false};

  Eigen::Map<Eigen::Matrix<double, kYRefSizePointMassMultiple,
                           kSamplesPointMassMultiple, Eigen::ColMajor>>
      acados_reference_states_{acados_in_pointmass_multiple.yref};
  Eigen::Map<Eigen::Matrix<double, kStateSizePointMassMultiple, 1,
                           Eigen::ColMajor>>
      acados_initial_state_{acados_in_pointmass_multiple.x0};
  Eigen::Map<Eigen::Matrix<double, kYRefSizePointMassMultiple, 1,
                           Eigen::ColMajor>>
      acados_reference_end_state_{acados_in_pointmass_multiple.yref_e};
  Eigen::Map<Eigen::Matrix<double, kStateSizePointMassMultiple,
                           kSamplesPointMassMultiple, Eigen::ColMajor>>
      acados_states_in_{acados_in_pointmass_multiple.x};
  Eigen::Map<Eigen::Matrix<double, kInputSizePointMassMultiple,
                           kSamplesPointMassMultiple, Eigen::ColMajor>>
      acados_inputs_in_{acados_in_pointmass_multiple.u};
  Eigen::Map<Eigen::Matrix<double, kStateSizePointMassMultiple,
                           kSamplesPointMassMultiple, Eigen::ColMajor>>
      acados_states_{acados_out_pointmass_multiple.x_out};
  Eigen::Map<Eigen::Matrix<double, kInputSizePointMassMultiple,
                           kSamplesPointMassMultiple, Eigen::ColMajor>>
      acados_inputs_{acados_out_pointmass_multiple.u_out};
  Eigen::Matrix<real_t, kInputSizePointMassMultiple, 1> kHoverInput_ =
      Eigen::Matrix<real_t, kInputSizePointMassMultiple, 1>::Zero();
};

} // namespace payload_pointmass_multiple_quadrotor

#endif
