#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>

#include <Eigen/Core>
#include <Eigen/LU>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mav_manager_srv/srv/lissajous.hpp>
#include <mav_manager_srv/srv/vec4.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <payload_pointmass_multiple_quadrotor/nmpc_planner_pointmass_multiple.h>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace payload_pointmass_multiple_quadrotor {

namespace {

constexpr int kRobots = 3;
constexpr int kPolyDegree = 9;

Eigen::Vector3d normalize(const Eigen::Vector3d &v) {
  const double n = v.norm();
  if (n < 1e-9) {
    throw std::runtime_error("Cannot normalize near-zero vector.");
  }
  return v / n;
}

Eigen::Matrix<double, 1, kPolyDegree + 1> derivativeRow(int deriv, double tau) {
  Eigen::Matrix<double, 1, kPolyDegree + 1> row;
  row.setZero();
  for (int k = deriv; k <= kPolyDegree; ++k) {
    double coeff = 1.0;
    for (int i = 0; i < deriv; ++i) {
      coeff *= static_cast<double>(k - i);
    }
    row(k) = coeff * std::pow(tau, static_cast<double>(k - deriv));
  }
  return row;
}

struct Poly9Eval {
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d j;
  Eigen::Vector3d s;
};

struct LissajousEval {
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d j;
  Eigen::Vector3d s;
};

struct CableState {
  double T;
  Eigen::Vector3d q;
  Eigen::Vector3d qdot;
  Eigen::Vector3d qddot;
};

CableState reconstructCable(const Eigen::Vector3d &lam,
                            const Eigen::Vector3d &lam_dot,
                            const Eigen::Vector3d &lam_ddot) {
  const double T = lam.norm();
  if (T < 1e-6) {
    throw std::runtime_error("Cable force magnitude is too small.");
  }

  CableState out;
  out.T = T;
  out.q = lam / T;
  const double Tdot = out.q.dot(lam_dot);
  out.qdot = (lam_dot - Tdot * out.q) / T;
  const double Tddot = out.q.dot(lam_ddot) + out.qdot.dot(lam_dot);
  out.qddot = (lam_ddot - Tddot * out.q - 2.0 * Tdot * out.qdot) / T;
  return out;
}

double clampDouble(double value, double lo, double hi) {
  return std::max(lo, std::min(value, hi));
}

} // namespace

class NMPCControlPointMassMultipleNodelet : public rclcpp::Node {
public:
  explicit NMPCControlPointMassMultipleNodelet(
      const rclcpp::NodeOptions &options)
      : Node("nmpc_control_pointmass_multiple_nodelet", options) {
    this->declare_parameter("mass_payload", 0.2);
    this->declare_parameter("mass", 1.0);
    this->declare_parameter("gravity", 9.81);
    this->declare_parameter("cable_length", 1.0);
    this->declare_parameter("nmpc.horizon_steps", 31);
    this->declare_parameter("nmpc.horizon_time", 1.5);
    this->declare_parameter<std::vector<double>>(
        "planner.goal", std::vector<double>{5.0, 1.2, 0.3});
    this->declare_parameter("planner.duration", 5.0);
    this->declare_parameter("planner.type", "lissajous");
    this->declare_parameter<std::vector<double>>(
        "planner.lissajous.offset",
        std::vector<double>{0.34049933598831467, -0.0007520805616463245,
                            0.8936953489145677});
    this->declare_parameter("planner.lissajous.x_amp", 3.5);
    this->declare_parameter("planner.lissajous.y_amp", 0.5);
    this->declare_parameter("planner.lissajous.z_amp", 0.2);
    this->declare_parameter("planner.lissajous.period", 6.5);
    this->declare_parameter("planner.lissajous.num_cycles", 4.0);
    this->declare_parameter("planner.lissajous.ramp_time", 6.0);
    this->declare_parameter("planner.lissajous.x_num_periods", 1.0);
    this->declare_parameter("planner.lissajous.y_num_periods", 2.0);
    this->declare_parameter("planner.lissajous.z_num_periods", 1.0);

    this->get_parameter("mass_payload", mass_payload_);
    this->get_parameter("mass", mass_quadrotor_);
    this->get_parameter("gravity", gravity_);
    this->get_parameter("cable_length", cable_length_);
    this->get_parameter("nmpc.horizon_steps", horizon_steps_);
    this->get_parameter("nmpc.horizon_time", horizon_time_);
    this->get_parameter("planner.duration", planner_duration_);
    this->get_parameter("planner.type", planner_type_);
    const auto goal = this->get_parameter("planner.goal").as_double_array();
    if (goal.size() == 3) {
      planner_goal_ << goal[0], goal[1], goal[2];
    }
    const auto lissajous_offset =
        this->get_parameter("planner.lissajous.offset").as_double_array();
    if (lissajous_offset.size() == 3) {
      lissajous_offset_ << lissajous_offset[0], lissajous_offset[1],
          lissajous_offset[2];
    }
    this->get_parameter("planner.lissajous.x_amp", lissajous_x_amp_);
    this->get_parameter("planner.lissajous.y_amp", lissajous_y_amp_);
    this->get_parameter("planner.lissajous.z_amp", lissajous_z_amp_);
    this->get_parameter("planner.lissajous.period", lissajous_period_);
    this->get_parameter("planner.lissajous.num_cycles", lissajous_num_cycles_);
    this->get_parameter("planner.lissajous.ramp_time", lissajous_ramp_time_);
    this->get_parameter("planner.lissajous.x_num_periods",
                        lissajous_x_num_periods_);
    this->get_parameter("planner.lissajous.y_num_periods",
                        lissajous_y_num_periods_);
    this->get_parameter("planner.lissajous.z_num_periods",
                        lissajous_z_num_periods_);
    ts_ = horizon_time_ / static_cast<double>(horizon_steps_);

    auto qos_profile = rclcpp::SensorDataQoS();

    pub_ref_traj_ = this->create_publisher<nav_msgs::msg::Path>(
        "payload_reference_path_pointmass_multiple", 1);
    pub_pred_traj_ = this->create_publisher<nav_msgs::msg::Path>(
        "payload_predicted_path_pointmass_multiple", 1);
    pub_desired_payload_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "payload_desired_odom_pointmass_multiple", 1);

    for (int i = 0; i < kRobots; ++i) {
      pub_desired_quadrotor_[i] =
          this->create_publisher<quadrotor_msgs::msg::PositionCommand>(
              "/quadrotor" + std::to_string(i + 1) +
                  "/payload_planner_quadrotor_cmd",
              1);
    }

    sub_payload_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/quadrotor1/payload/odom", qos_profile,
        std::bind(&NMPCControlPointMassMultipleNodelet::payloadOdomCallback,
                  this, std::placeholders::_1));

    for (int i = 0; i < kRobots; ++i) {
      sub_quad_odometry_[i] =
          this->create_subscription<nav_msgs::msg::Odometry>(
              "/quadrotor" + std::to_string(i + 1) + "/odom", qos_profile,
              [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
                quadOdomCallback(i, msg);
              });
    }

    srv_full_planner_ = this->create_service<std_srvs::srv::SetBool>(
        "full_planner",
        std::bind(&NMPCControlPointMassMultipleNodelet::fullPlannerCallback,
                  this, std::placeholders::_1, std::placeholders::_2));
    srv_set_full_planner_point_ = this->create_service<
        mav_manager_srv::srv::Vec4>(
        "set_full_planner_point",
        std::bind(
            &NMPCControlPointMassMultipleNodelet::setFullPlannerPointCallback,
            this, std::placeholders::_1, std::placeholders::_2));
    srv_set_full_planner_lissajous_ =
        this->create_service<mav_manager_srv::srv::Lissajous>(
            "set_full_planner_lissajous",
            std::bind(&NMPCControlPointMassMultipleNodelet::
                          setFullPlannerLissajousCallback,
                      this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void buildPlanner(const Eigen::Vector3d &p0) {
    const auto planner_build_start = std::chrono::steady_clock::now();
    planner_start_ = p0;

    const Eigen::Vector3d equilibrium_payload =
        planner_type_ == "lissajous" ? lissajous_offset_ : planner_start_;

    for (int i = 0; i < kRobots; ++i) {
      q_eq_[i] = normalize(equilibrium_payload - quad_position_[i]);
      alpha_(i) = 1.0 / 3.0;
    }

    Eigen::Matrix3d Q;
    Q.col(0) = q_eq_[0];
    Q.col(1) = q_eq_[1];
    Q.col(2) = q_eq_[2];
    const Eigen::Vector3d rhs(0.0, 0.0, -mass_payload_ * gravity_);
    const Eigen::Vector3d tensions = Q.fullPivLu().solve(rhs);
    for (int i = 0; i < kRobots; ++i) {
      lambda_eq_[i] = tensions(i) * q_eq_[i];
    }

    if (planner_type_ == "lissajous") {
      buildLissajousPlanner();
      reference_start_time_ = this->now().seconds();
      planner_initialized_ = true;
      const auto planner_build_end = std::chrono::steady_clock::now();
      const double planner_build_ms =
          std::chrono::duration<double, std::milli>(planner_build_end -
                                                    planner_build_start)
              .count();
      RCLCPP_INFO(
          this->get_logger(),
          "[PointMassMultiple NMPC] planner initialization took %.3f ms",
          planner_build_ms);
      return;
    }

    Eigen::Matrix<double, 10, 10> A;
    A.setZero();
    for (int r = 0; r < 5; ++r) {
      A.row(r) = derivativeRow(r, 0.0);
      A.row(5 + r) = derivativeRow(r, 1.0);
    }

    Eigen::Matrix<double, 10, 3> b;
    b.setZero();
    b.row(0) = planner_start_;
    b.row(5) = planner_goal_;
    poly_coeffs_ = A.fullPivLu().solve(b);

    reference_start_time_ = this->now().seconds();
    planner_initialized_ = true;
    const auto planner_build_end = std::chrono::steady_clock::now();
    const double planner_build_ms = std::chrono::duration<double, std::milli>(
                                        planner_build_end - planner_build_start)
                                        .count();
    RCLCPP_INFO(this->get_logger(),
                "[PointMassMultiple NMPC] planner initialization took %.3f ms",
                planner_build_ms);
  }

  void fullPlannerCallback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    use_full_planner_ = request->data;
    if (use_full_planner_ && have_payload_odom_) {
      restartPlannerFromCurrentPayload();
    }
    response->success = true;
    response->message = use_full_planner_
                            ? "Full point-mass multiple planner active"
                            : "Full point-mass multiple planner inactive";
    RCLCPP_INFO(this->get_logger(), "[PointMassMultiple NMPC] %s",
                response->message.c_str());
  }

  void setFullPlannerPointCallback(
      const std::shared_ptr<mav_manager_srv::srv::Vec4::Request> request,
      std::shared_ptr<mav_manager_srv::srv::Vec4::Response> response) {
    planner_type_ = "point_to_point";
    planner_goal_ << request->goal[0], request->goal[1], request->goal[2];
    if (request->goal[3] > 0.0F) {
      planner_duration_ = request->goal[3];
    }

    if (have_payload_odom_) {
      restartPlannerFromCurrentPayload();
      response->success = true;
      response->message = "Full planner point-to-point goal set to [" +
                          std::to_string(planner_goal_.x()) + ", " +
                          std::to_string(planner_goal_.y()) + ", " +
                          std::to_string(planner_goal_.z()) + "]";
    } else {
      planner_initialized_ = false;
      response->success = false;
      response->message =
          "Point-to-point goal set, but no payload odom has been received yet";
    }
    RCLCPP_INFO(this->get_logger(), "[PointMassMultiple NMPC] %s",
                response->message.c_str());
  }

  void setFullPlannerLissajousCallback(
      const std::shared_ptr<mav_manager_srv::srv::Lissajous::Request> request,
      std::shared_ptr<mav_manager_srv::srv::Lissajous::Response> response) {
    if (request->period <= 0.0F || request->num_cycles <= 0.0F ||
        request->ramp_time <= 0.0F) {
      response->success = false;
      response->message =
          "Lissajous period, num_cycles, and ramp_time must be positive";
      return;
    }

    planner_type_ = "lissajous";
    if (have_payload_odom_) {
      lissajous_offset_ = current_state_.segment<3>(0);
    }
    lissajous_x_amp_ = request->x_amp;
    lissajous_y_amp_ = request->y_amp;
    lissajous_z_amp_ = request->z_amp;
    lissajous_period_ = request->period;
    lissajous_num_cycles_ = request->num_cycles;
    lissajous_ramp_time_ = request->ramp_time;
    lissajous_x_num_periods_ = request->x_num_periods;
    lissajous_y_num_periods_ = request->y_num_periods;
    lissajous_z_num_periods_ = request->z_num_periods;

    if (have_payload_odom_) {
      restartPlannerFromCurrentPayload();
      response->success = true;
      response->message = "Full planner Lissajous trajectory set and restarted";
    } else {
      planner_initialized_ = false;
      response->success = false;
      response->message =
          "Lissajous trajectory set, but no payload odom has been received yet";
    }
    RCLCPP_INFO(this->get_logger(), "[PointMassMultiple NMPC] %s",
                response->message.c_str());
  }

  void restartPlannerFromCurrentPayload() {
    if (!(have_quad_odom_[0] && have_quad_odom_[1] && have_quad_odom_[2])) {
      RCLCPP_WARN(this->get_logger(),
                  "[PointMassMultiple NMPC] Cannot restart planner before all "
                  "quadrotor odometries are received.");
      return;
    }
    planner_initialized_ = false;
    buildPlanner(current_state_.segment<3>(0));
    reference_start_time_ = this->now().seconds();
    buildReferenceHorizon(0.0);
  }

  void buildLissajousPlanner() {
    const double tr = lissajous_ramp_time_;
    const double tr2 = tr * tr;
    const double tr3 = tr2 * tr;
    const double tr4 = tr3 * tr;
    const double tr5 = tr4 * tr;
    const double tr6 = tr5 * tr;
    const double tr7 = tr6 * tr;
    const double tr8 = tr7 * tr;

    lissajous_a4_ = 35.0 / tr4;
    lissajous_a5_ = -84.0 / tr5;
    lissajous_a6_ = 70.0 / tr6;
    lissajous_a7_ = -20.0 / tr7;
    lissajous_total_s_ = lissajous_period_ * lissajous_num_cycles_;
    lissajous_ramp_s_ = lissajous_a7_ * tr8 / 8.0 + lissajous_a6_ * tr7 / 7.0 +
                        lissajous_a5_ * tr6 / 6.0 + lissajous_a4_ * tr5 / 5.0;
    lissajous_const_time_ = lissajous_total_s_ - 2.0 * lissajous_ramp_s_;
    if (lissajous_const_time_ < 0.0) {
      throw std::runtime_error("planner.lissajous.ramp_time is too large for "
                               "the selected period and num_cycles.");
    }
    planner_duration_ = 2.0 * tr + lissajous_const_time_;
  }

  Poly9Eval evalPoly9(double t) const {
    const double tau = std::clamp(t / planner_duration_, 0.0, 1.0);
    std::array<Eigen::Vector3d, 6> vals;
    for (int r = 0; r < 6; ++r) {
      vals[r] = derivativeRow(r, tau) * poly_coeffs_;
    }
    return Poly9Eval{
        vals[0],
        vals[1] / planner_duration_,
        vals[2] / std::pow(planner_duration_, 2.0),
        vals[3] / std::pow(planner_duration_, 3.0),
        vals[4] / std::pow(planner_duration_, 4.0),
    };
  }

  LissajousEval evalLissajous(double t) const {
    const double tc = clampDouble(t, 0.0, planner_duration_);
    double s = 0.0;
    double sdot = 0.0;
    double sddot = 0.0;
    double sdddot = 0.0;
    double sddddot = 0.0;

    if (tc < lissajous_ramp_time_) {
      const double t2 = tc * tc;
      const double t3 = t2 * tc;
      const double t4 = t3 * tc;
      const double t5 = t4 * tc;
      const double t6 = t5 * tc;
      const double t7 = t6 * tc;
      const double t8 = t7 * tc;
      s = lissajous_a7_ * t8 / 8.0 + lissajous_a6_ * t7 / 7.0 +
          lissajous_a5_ * t6 / 6.0 + lissajous_a4_ * t5 / 5.0;
      sdot = lissajous_a7_ * t7 + lissajous_a6_ * t6 + lissajous_a5_ * t5 +
             lissajous_a4_ * t4;
      sddot = 7.0 * lissajous_a7_ * t6 + 6.0 * lissajous_a6_ * t5 +
              5.0 * lissajous_a5_ * t4 + 4.0 * lissajous_a4_ * t3;
      sdddot = 42.0 * lissajous_a7_ * t5 + 30.0 * lissajous_a6_ * t4 +
               20.0 * lissajous_a5_ * t3 + 12.0 * lissajous_a4_ * t2;
      sddddot = 210.0 * lissajous_a7_ * t4 + 120.0 * lissajous_a6_ * t3 +
                60.0 * lissajous_a5_ * t2 + 24.0 * lissajous_a4_ * tc;
    } else if (tc < planner_duration_ - lissajous_ramp_time_) {
      s = lissajous_ramp_s_ + tc - lissajous_ramp_time_;
      sdot = 1.0;
    } else {
      const double te = planner_duration_ - tc;
      const double te2 = te * te;
      const double te3 = te2 * te;
      const double te4 = te3 * te;
      const double te5 = te4 * te;
      const double te6 = te5 * te;
      const double te7 = te6 * te;
      const double te8 = te7 * te;
      s = 2.0 * lissajous_ramp_s_ + lissajous_const_time_ -
          lissajous_a7_ * te8 / 8.0 - lissajous_a6_ * te7 / 7.0 -
          lissajous_a5_ * te6 / 6.0 - lissajous_a4_ * te5 / 5.0;
      sdot = lissajous_a7_ * te7 + lissajous_a6_ * te6 + lissajous_a5_ * te5 +
             lissajous_a4_ * te4;
      sddot = -7.0 * lissajous_a7_ * te6 - 6.0 * lissajous_a6_ * te5 -
              5.0 * lissajous_a5_ * te4 - 4.0 * lissajous_a4_ * te3;
      sdddot = 42.0 * lissajous_a7_ * te5 + 30.0 * lissajous_a6_ * te4 +
               20.0 * lissajous_a5_ * te3 + 12.0 * lissajous_a4_ * te2;
      sddddot = -210.0 * lissajous_a7_ * te4 - 120.0 * lissajous_a6_ * te3 -
                60.0 * lissajous_a5_ * te2 - 24.0 * lissajous_a4_ * te;
    }

    const double x_coeff =
        2.0 * M_PI * lissajous_x_num_periods_ / lissajous_period_;
    const double y_coeff =
        2.0 * M_PI * lissajous_y_num_periods_ / lissajous_period_;
    const double z_coeff =
        2.0 * M_PI * lissajous_z_num_periods_ / lissajous_period_;

    const double x_coeff2 = x_coeff * x_coeff;
    const double x_coeff3 = x_coeff2 * x_coeff;
    const double x_coeff4 = x_coeff3 * x_coeff;
    const double y_coeff2 = y_coeff * y_coeff;
    const double y_coeff3 = y_coeff2 * y_coeff;
    const double y_coeff4 = y_coeff3 * y_coeff;
    const double z_coeff2 = z_coeff * z_coeff;
    const double z_coeff3 = z_coeff2 * z_coeff;
    const double z_coeff4 = z_coeff3 * z_coeff;

    LissajousEval out;
    out.p(0) = lissajous_x_amp_ * (1.0 - std::cos(x_coeff * s));
    out.p(1) = lissajous_y_amp_ * std::sin(y_coeff * s);
    out.p(2) = lissajous_z_amp_ * std::sin(z_coeff * s);

    out.v(0) = lissajous_x_amp_ * x_coeff * std::sin(x_coeff * s) * sdot;
    out.v(1) = lissajous_y_amp_ * y_coeff * std::cos(y_coeff * s) * sdot;
    out.v(2) = lissajous_z_amp_ * z_coeff * std::cos(z_coeff * s) * sdot;

    out.a(0) =
        lissajous_x_amp_ * (x_coeff2 * std::cos(x_coeff * s) * sdot * sdot +
                            x_coeff * std::sin(x_coeff * s) * sddot);
    out.a(1) =
        lissajous_y_amp_ * (-y_coeff2 * std::sin(y_coeff * s) * sdot * sdot +
                            y_coeff * std::cos(y_coeff * s) * sddot);
    out.a(2) =
        lissajous_z_amp_ * (-z_coeff2 * std::sin(z_coeff * s) * sdot * sdot +
                            z_coeff * std::cos(z_coeff * s) * sddot);

    out.j(0) = lissajous_x_amp_ *
               (-x_coeff3 * std::sin(x_coeff * s) * std::pow(sdot, 3.0) +
                3.0 * x_coeff2 * std::cos(x_coeff * s) * sdot * sddot +
                x_coeff * std::sin(x_coeff * s) * sdddot);
    out.j(1) = lissajous_y_amp_ *
               (-y_coeff3 * std::cos(y_coeff * s) * std::pow(sdot, 3.0) -
                3.0 * y_coeff2 * std::sin(y_coeff * s) * sdot * sddot +
                y_coeff * std::cos(y_coeff * s) * sdddot);
    out.j(2) = lissajous_z_amp_ *
               (-z_coeff3 * std::cos(z_coeff * s) * std::pow(sdot, 3.0) -
                3.0 * z_coeff2 * std::sin(z_coeff * s) * sdot * sddot +
                z_coeff * std::cos(z_coeff * s) * sdddot);

    out.s(0) = lissajous_x_amp_ *
               (-x_coeff4 * std::cos(x_coeff * s) * std::pow(sdot, 4.0) +
                3.0 * x_coeff2 * std::cos(x_coeff * s) * sddot * sddot +
                x_coeff * std::sin(x_coeff * s) * sddddot -
                6.0 * x_coeff3 * std::sin(x_coeff * s) * sddot * sdot * sdot +
                4.0 * x_coeff2 * std::cos(x_coeff * s) * sdddot * sdot);
    out.s(1) = lissajous_y_amp_ *
               (y_coeff4 * std::sin(y_coeff * s) * std::pow(sdot, 4.0) -
                3.0 * y_coeff2 * std::sin(y_coeff * s) * sddot * sddot +
                y_coeff * std::cos(y_coeff * s) * sddddot -
                6.0 * y_coeff3 * std::cos(y_coeff * s) * sddot * sdot * sdot -
                4.0 * y_coeff2 * std::sin(y_coeff * s) * sdddot * sdot);
    out.s(2) = lissajous_z_amp_ *
               (z_coeff4 * std::sin(z_coeff * s) * std::pow(sdot, 4.0) -
                3.0 * z_coeff2 * std::sin(z_coeff * s) * sddot * sddot +
                z_coeff * std::cos(z_coeff * s) * sddddot -
                6.0 * z_coeff3 * std::cos(z_coeff * s) * sddot * sdot * sdot -
                4.0 * z_coeff2 * std::sin(z_coeff * s) * sdddot * sdot);

    out.p += lissajous_offset_;
    return out;
  }

  void buildReferenceHorizon(double elapsed) {
    for (int stage = 0; stage < kSamplesPointMassMultiple; ++stage) {
      const double t_stage = std::min(
          elapsed + static_cast<double>(stage) * ts_, planner_duration_);
      Eigen::Vector3d p;
      Eigen::Vector3d v;
      Eigen::Vector3d a;
      Eigen::Vector3d j;
      Eigen::Vector3d s;
      if (planner_type_ == "lissajous") {
        const LissajousEval eval = evalLissajous(t_stage);
        p = eval.p;
        v = eval.v;
        a = eval.a;
        j = eval.j;
        s = eval.s;
      } else {
        const Poly9Eval eval = evalPoly9(t_stage);
        p = eval.p;
        v = eval.v;
        a = eval.a;
        j = eval.j;
        s = eval.s;
      }

      reference_states_.block<3, 1>(0, stage) = p;
      reference_states_.block<3, 1>(3, stage) = v;

      for (int i = 0; i < kRobots; ++i) {
        const Eigen::Vector3d lambda =
            lambda_eq_[i] - alpha_(i) * mass_payload_ * a;
        const Eigen::Vector3d lambda_dot = -alpha_(i) * mass_payload_ * j;
        const Eigen::Vector3d lambda_ddot = -alpha_(i) * mass_payload_ * s;
        const CableState cable =
            reconstructCable(lambda, lambda_dot, lambda_ddot);

        reference_states_.block<3, 1>(6 + 3 * i, stage) = cable.q;
        reference_states_.block<3, 1>(15 + 3 * i, stage) =
            cable.q.cross(cable.qdot);
        reference_inputs_.block<3, 1>(3 * i, stage) =
            a - cable_length_ * cable.qddot;
      }
    }

    controller_.setReferenceStates(reference_states_);
    controller_.setReferenceInputs(reference_inputs_);
  }

  Eigen::Vector3d quadrotorPositionFromPayloadState(
      const Eigen::Ref<
          const Eigen::Matrix<double, kStateSizePointMassMultiple, 1>> &state,
      int quad_idx) const {
    const int n_offset = 6 + 3 * quad_idx;
    return state.segment<3>(0) - cable_length_ * state.segment<3>(n_offset);
  }

  Eigen::Vector3d quadrotorVelocityFromPayloadState(
      const Eigen::Ref<
          const Eigen::Matrix<double, kStateSizePointMassMultiple, 1>> &state,
      int quad_idx) const {
    const int n_offset = 6 + 3 * quad_idx;
    const int r_offset = 15 + 3 * quad_idx;
    return state.segment<3>(3) -
           cable_length_ *
               state.segment<3>(r_offset).cross(state.segment<3>(n_offset));
  }

  Eigen::Vector3d quadrotorAccelerationFromInput(
      const Eigen::Ref<
          const Eigen::Matrix<double, kInputSizePointMassMultiple, 1>> &input,
      int quad_idx) const {
    return input.segment<3>(3 * quad_idx);
  }

  double tensionFromStateInput(
      const Eigen::Ref<
          const Eigen::Matrix<double, kStateSizePointMassMultiple, 1>> &state,
      const Eigen::Ref<
          const Eigen::Matrix<double, kInputSizePointMassMultiple, 1>> &input,
      int quad_idx) const {
    Eigen::Matrix3d N;
    Eigen::Vector3d d;
    for (int i = 0; i < kRobots; ++i) {
      const int n_offset = 6 + 3 * i;
      const int r_offset = 15 + 3 * i;
      const Eigen::Vector3d n = state.segment<3>(n_offset);
      const Eigen::Vector3d r = state.segment<3>(r_offset);
      const Eigen::Vector3d a_q = input.segment<3>(3 * i);
      N.col(i) = n;
      d(i) = n.dot(a_q) - cable_length_ * r.squaredNorm();
    }

    Eigen::Matrix<double, 6, 6> K = Eigen::Matrix<double, 6, 6>::Zero();
    K.block<3, 3>(0, 0) = mass_payload_ * Eigen::Matrix3d::Identity();
    K.block<3, 3>(0, 3) = N;
    K.block<3, 3>(3, 0) = N.transpose();

    Eigen::Matrix<double, 6, 1> rhs;
    rhs.setZero();
    rhs.segment<3>(0) =
        -mass_payload_ * gravity_ * Eigen::Vector3d(0.0, 0.0, 1.0);
    rhs.segment<3>(3) = d;

    const Eigen::Matrix<double, 6, 1> solution = K.fullPivLu().solve(rhs);
    return solution(3 + quad_idx);
  }

  Eigen::Quaterniond desiredOrientationFromForce(const Eigen::Vector3d &force,
                                                 double yaw) const {
    const Eigen::Vector3d Yc(-std::sin(yaw), std::cos(yaw), 0.0);
    Eigen::Vector3d Xb = Yc.cross(force);
    if (Xb.norm() < 1e-9) {
      Xb = Eigen::Vector3d::UnitX();
    } else {
      Xb.normalize();
    }

    Eigen::Vector3d Yb = force.cross(Xb);
    if (Yb.norm() < 1e-9) {
      Yb = Eigen::Vector3d::UnitY();
    } else {
      Yb.normalize();
    }

    const Eigen::Vector3d Zb = Xb.cross(Yb);
    Eigen::Matrix3d R;
    R.col(0) = Xb;
    R.col(1) = Yb;
    R.col(2) = Zb;
    return Eigen::Quaterniond(R).normalized();
  }

  void quadOdomCallback(int quad_idx,
                        const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    quad_position_[quad_idx] << odom_msg->pose.pose.position.x,
        odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z;
    quad_velocity_[quad_idx] << odom_msg->twist.twist.linear.x,
        odom_msg->twist.twist.linear.y, odom_msg->twist.twist.linear.z;
    have_quad_odom_[quad_idx] = true;
  }

  void payloadOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    frame_id_ = odom_msg->header.frame_id;
    current_state_.setZero();
    current_state_(0) = odom_msg->pose.pose.position.x;
    current_state_(1) = odom_msg->pose.pose.position.y;
    current_state_(2) = odom_msg->pose.pose.position.z;
    current_state_(3) = odom_msg->twist.twist.linear.x;
    current_state_(4) = odom_msg->twist.twist.linear.y;
    current_state_(5) = odom_msg->twist.twist.linear.z;

    const Eigen::Vector3d payload_position = current_state_.segment<3>(0);
    const Eigen::Vector3d payload_velocity = current_state_.segment<3>(3);

    for (int i = 0; i < kRobots; ++i) {
      const Eigen::Vector3d a = payload_position - quad_position_[i];
      const Eigen::Vector3d a_dot = payload_velocity - quad_velocity_[i];
      const double norm_a = std::max(a.norm(), 1e-9);
      const double dot_a = std::max(a.dot(a), 1e-12);
      const Eigen::Matrix3d projection =
          Eigen::Matrix3d::Identity() - (a * a.transpose()) / dot_a;
      const Eigen::Vector3d n_dot = (1.0 / norm_a) * projection * a_dot;
      const Eigen::Vector3d cable_dir = a / norm_a;
      const Eigen::Vector3d cable_r = cable_dir.cross(n_dot);
      current_state_.segment<3>(6 + 3 * i) = cable_dir;
      current_state_.segment<3>(15 + 3 * i) = cable_r;
    }

    const double stamp_sec =
        static_cast<double>(odom_msg->header.stamp.sec) +
        static_cast<double>(odom_msg->header.stamp.nanosec) * 1e-9;
    controller_.setState(current_state_, stamp_sec);
    have_payload_odom_ = true;

    if (!planner_initialized_ && use_full_planner_) {
      buildPlanner(payload_position);
    }

    run();
  }

  void publishPrediction() {
    const auto predicted_states = controller_.getPredictedStates();
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = frame_id_;
    for (int i = 0; i < kSamplesPointMassMultiple; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = predicted_states(0, i);
      pose.pose.position.y = predicted_states(1, i);
      pose.pose.position.z = predicted_states(2, i);
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }
    pub_pred_traj_->publish(path_msg);
  }

  void publishReference() {
    const auto reference_states = controller_.getReferenceStates();
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = frame_id_;
    for (int i = 0; i < kSamplesPointMassMultiple; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = reference_states(0, i);
      pose.pose.position.y = reference_states(1, i);
      pose.pose.position.z = reference_states(2, i);
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }
    pub_ref_traj_->publish(path_msg);
  }

  void publishDesiredPayloadOdom() {
    const auto reference_states = controller_.getReferenceStates();
    const int ref_idx = std::min(1, kSamplesPointMassMultiple - 1);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = frame_id_;
    odom_msg.pose.pose.position.x = reference_states(0, ref_idx);
    odom_msg.pose.pose.position.y = reference_states(1, ref_idx);
    odom_msg.pose.pose.position.z = reference_states(2, ref_idx);
    odom_msg.pose.pose.orientation.w = 1.0;
    odom_msg.twist.twist.linear.x = reference_states(3, ref_idx);
    odom_msg.twist.twist.linear.y = reference_states(4, ref_idx);
    odom_msg.twist.twist.linear.z = reference_states(5, ref_idx);
    pub_desired_payload_odom_->publish(odom_msg);
  }

  void publishDesiredQuadrotorCommand() {
    const auto predicted_states = controller_.getPredictedStates();
    const auto predicted_inputs = controller_.getPredictedInputs();

    const int first_state_idx = std::min(1, kSamplesPointMassMultiple - 1);
    const auto first_state = predicted_states.col(first_state_idx);
    const auto first_input = predicted_inputs.col(0);

    for (int quad_idx = 0; quad_idx < kRobots; ++quad_idx) {
      quadrotor_msgs::msg::PositionCommand msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = frame_id_;
      msg.planner_type = quadrotor_msgs::msg::PositionCommand::PAYLOAD_PLANNER;

      const Eigen::Vector3d quad_pos =
          quadrotorPositionFromPayloadState(first_state, quad_idx);
      const Eigen::Vector3d quad_vel =
          quadrotorVelocityFromPayloadState(first_state, quad_idx);
      const Eigen::Vector3d quad_acc =
          quadrotorAccelerationFromInput(first_input, quad_idx);
      const Eigen::Vector3d cable_dir =
          first_state.segment<3>(6 + 3 * quad_idx);
      const double tension =
          tensionFromStateInput(first_state, first_input, quad_idx);
      const Eigen::Vector3d cable_force = tension * cable_dir;

      msg.position.x = quad_pos(0);
      msg.position.y = quad_pos(1);
      msg.position.z = quad_pos(2);
      msg.velocity.x = quad_vel(0);
      msg.velocity.y = quad_vel(1);
      msg.velocity.z = quad_vel(2);
      msg.acceleration.x = quad_acc(0);
      msg.acceleration.y = quad_acc(1);
      msg.acceleration.z = quad_acc(2);
      msg.tension = tension;
      msg.cable_direction.x = cable_dir(0);
      msg.cable_direction.y = cable_dir(1);
      msg.cable_direction.z = cable_dir(2);
      msg.cable_force.x = cable_force(0);
      msg.cable_force.y = cable_force(1);
      msg.cable_force.z = cable_force(2);

      for (int i = 0; i < kSamplesPointMassMultiple; ++i) {
        const int state_idx = std::min(i + 1, kSamplesPointMassMultiple - 1);
        const int input_idx = std::min(i, kSamplesPointMassMultiple - 1);
        const auto state_i = predicted_states.col(state_idx);
        const auto input_i = predicted_inputs.col(input_idx);

        const Eigen::Vector3d quad_position =
            quadrotorPositionFromPayloadState(state_i, quad_idx);
        const Eigen::Vector3d quad_velocity =
            quadrotorVelocityFromPayloadState(state_i, quad_idx);
        const Eigen::Vector3d quad_acceleration =
            quadrotorAccelerationFromInput(input_i, quad_idx);
        const Eigen::Vector3d cable_direction =
            state_i.segment<3>(6 + 3 * quad_idx);
        const double point_tension =
            tensionFromStateInput(state_i, input_i, quad_idx);
        const Eigen::Vector3d desired_force =
            mass_quadrotor_ * quad_acceleration +
            mass_quadrotor_ * gravity_ * Eigen::Vector3d::UnitZ() -
            point_tension * cable_direction;
        const Eigen::Quaterniond orientation =
            desiredOrientationFromForce(desired_force, 0.0);
        const Eigen::Vector3d body_z = orientation.toRotationMatrix().col(2);
        const double thrust = body_z.dot(desired_force);

        quadrotor_msgs::msg::TrajectoryPoint point;
        point.position.x = quad_position(0);
        point.position.y = quad_position(1);
        point.position.z = quad_position(2);
        point.velocity.x = quad_velocity(0);
        point.velocity.y = quad_velocity(1);
        point.velocity.z = quad_velocity(2);
        point.acceleration.x = quad_acceleration(0);
        point.acceleration.y = quad_acceleration(1);
        point.acceleration.z = quad_acceleration(2);
        point.position_quad.x = quad_position(0);
        point.position_quad.y = quad_position(1);
        point.position_quad.z = quad_position(2);
        point.velocity_quad.x = quad_velocity(0);
        point.velocity_quad.y = quad_velocity(1);
        point.velocity_quad.z = quad_velocity(2);
        point.acceleration_quad.x = quad_acceleration(0);
        point.acceleration_quad.y = quad_acceleration(1);
        point.acceleration_quad.z = quad_acceleration(2);
        point.tension = point_tension;
        point.cable_direction.x = cable_direction(0);
        point.cable_direction.y = cable_direction(1);
        point.cable_direction.z = cable_direction(2);
        point.cable_r.x = state_i(15 + 3 * quad_idx);
        point.cable_r.y = state_i(16 + 3 * quad_idx);
        point.cable_r.z = state_i(17 + 3 * quad_idx);
        point.force = thrust;
        point.quaternion.w = orientation.w();
        point.quaternion.x = orientation.x();
        point.quaternion.y = orientation.y();
        point.quaternion.z = orientation.z();
        msg.points.push_back(point);
      }

      pub_desired_quadrotor_[quad_idx]->publish(msg);
    }
  }

  void run() {
    if (!use_full_planner_) {
      return;
    }
    if (!have_payload_odom_ || !planner_initialized_) {
      return;
    }
    if (!(have_quad_odom_[0] && have_quad_odom_[1] && have_quad_odom_[2])) {
      return;
    }

    const double now_sec = this->now().seconds();
    const double elapsed =
        std::min(now_sec - reference_start_time_, planner_duration_);

    const auto reference_start = std::chrono::steady_clock::now();
    buildReferenceHorizon(elapsed);
    const auto reference_end = std::chrono::steady_clock::now();
    const double reference_ms = std::chrono::duration<double, std::milli>(
                                    reference_end - reference_start)
                                    .count();

    const auto solve_start = std::chrono::steady_clock::now();
    const int acados_status = controller_.run();
    const auto solve_end = std::chrono::steady_clock::now();
    const double solve_ms =
        std::chrono::duration<double, std::milli>(solve_end - solve_start)
            .count();
    if (acados_status != 0) {
      RCLCPP_WARN(this->get_logger(),
                  "[PointMassMultiple NMPC] acados returned %d", acados_status);
      return;
    }

    const auto publish_start = std::chrono::steady_clock::now();
    publishPrediction();
    publishReference();
    publishDesiredPayloadOdom();
    publishDesiredQuadrotorCommand();
    const auto publish_end = std::chrono::steady_clock::now();
    const double publish_ms =
        std::chrono::duration<double, std::milli>(publish_end - publish_start)
            .count();

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[PointMassMultiple NMPC] reference horizon: %.3f ms | solve: %.3f ms "
        "| publish ref/pred/cmd: %.3f ms",
        reference_ms, solve_ms, publish_ms);
  }

  NMPCControlPointMassMultiple controller_;
  std::string frame_id_{"world"};
  double mass_payload_{0.2};
  double mass_quadrotor_{1.0};
  double gravity_{9.81};
  double cable_length_{1.0};
  int horizon_steps_{31};
  double horizon_time_{1.5};
  double ts_{horizon_time_ / static_cast<double>(horizon_steps_)};
  double planner_duration_{5.0};
  std::string planner_type_{"point_to_point"};
  double reference_start_time_{0.0};
  bool have_payload_odom_{false};
  bool planner_initialized_{false};
  bool use_full_planner_{false};
  Eigen::Vector3d planner_start_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d planner_goal_{5.0, 1.2, 0.3};
  Eigen::Vector3d lissajous_offset_{0.34049933598831467, -0.0007520805616463245,
                                    0.8936953489145677};
  double lissajous_x_amp_{3.5};
  double lissajous_y_amp_{0.5};
  double lissajous_z_amp_{0.2};
  double lissajous_period_{4.0};
  double lissajous_num_cycles_{4.0};
  double lissajous_ramp_time_{6.0};
  double lissajous_x_num_periods_{1.0};
  double lissajous_y_num_periods_{2.0};
  double lissajous_z_num_periods_{1.0};
  double lissajous_total_s_{0.0};
  double lissajous_ramp_s_{0.0};
  double lissajous_const_time_{0.0};
  double lissajous_a4_{0.0};
  double lissajous_a5_{0.0};
  double lissajous_a6_{0.0};
  double lissajous_a7_{0.0};
  Eigen::Matrix<double, 10, 3> poly_coeffs_{
      Eigen::Matrix<double, 10, 3>::Zero()};
  Eigen::Vector3d alpha_{Eigen::Vector3d::Constant(1.0 / 3.0)};
  std::array<Eigen::Vector3d, kRobots> q_eq_{};
  std::array<Eigen::Vector3d, kRobots> lambda_eq_{};
  Eigen::Matrix<double, kStateSizePointMassMultiple, 1> current_state_{
      Eigen::Matrix<double, kStateSizePointMassMultiple, 1>::Zero()};
  Eigen::Matrix<double, kStateSizePointMassMultiple, kSamplesPointMassMultiple>
      reference_states_{Eigen::Matrix<double, kStateSizePointMassMultiple,
                                      kSamplesPointMassMultiple>::Zero()};
  Eigen::Matrix<double, kInputSizePointMassMultiple, kSamplesPointMassMultiple>
      reference_inputs_{Eigen::Matrix<double, kInputSizePointMassMultiple,
                                      kSamplesPointMassMultiple>::Zero()};
  std::array<Eigen::Vector3d, kRobots> quad_position_{Eigen::Vector3d::Zero(),
                                                      Eigen::Vector3d::Zero(),
                                                      Eigen::Vector3d::Zero()};
  std::array<Eigen::Vector3d, kRobots> quad_velocity_{Eigen::Vector3d::Zero(),
                                                      Eigen::Vector3d::Zero(),
                                                      Eigen::Vector3d::Zero()};
  std::array<bool, kRobots> have_quad_odom_{false, false, false};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_ref_traj_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_pred_traj_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
      pub_desired_payload_odom_;
  std::array<rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr,
             kRobots>
      pub_desired_quadrotor_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      sub_payload_odometry_;
  std::array<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr, kRobots>
      sub_quad_odometry_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_full_planner_;
  rclcpp::Service<mav_manager_srv::srv::Vec4>::SharedPtr
      srv_set_full_planner_point_;
  rclcpp::Service<mav_manager_srv::srv::Lissajous>::SharedPtr
      srv_set_full_planner_lissajous_;
};

} // namespace payload_pointmass_multiple_quadrotor

RCLCPP_COMPONENTS_REGISTER_NODE(
    payload_pointmass_multiple_quadrotor::NMPCControlPointMassMultipleNodelet)
