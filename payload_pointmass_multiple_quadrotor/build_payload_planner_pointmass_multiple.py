#!/usr/bin/env python3
from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import List

import casadi as ca
import numpy as np
import yaml
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r", encoding="utf-8") as stream:
        parsed_yaml = yaml.safe_load(stream)
    if "/**" in parsed_yaml:
        parsed_yaml = parsed_yaml["/**"]["ros__parameters"]
    return parsed_yaml


class PayloadPointMassMultipleBuilder:
    def __init__(self, params):
        self.robot_num = 3

        self.t_N = float(params["nmpc"]["horizon_time"])
        self.N_prediction = int(params["nmpc"]["horizon_steps"])
        self.ts = self.t_N / self.N_prediction

        self.mass = float(params["mass_payload"])
        self.mass_quad = float(params.get("mass", 1.24))
        self.gravity = float(params["gravity"])
        self.length = float(params["cable_length"])
        self.e3 = ca.DM([0.0, 0.0, 1.0])

        self.norm_constraint_slack_weight = 10.0
        self.unit_vector_norm_tol = 1e-3

        self.Kp = np.diag([250.0, 250.0, 250.0])
        self.Kv = np.diag([1.0, 1.0, 1.0])
        self.Kp_n = np.diag([30.0, 30.0, 30.0])
        self.Kp_r = np.diag([1.0, 1.0, 1.0])
        self.R_q = np.diag([0.1, 0.1, 0.1])

        pos_0 = np.array(
            [0.34049933598831467, -0.0007520805616463245, 0.8936953489145677],
            dtype=np.double,
        )
        vel_0 = np.zeros((3,), dtype=np.double)

        pos_quad_1 = np.array(
            [-0.0029774502873919804, -0.30020808379855246, 1.4896822896707809],
            dtype=np.double,
        )
        pos_quad_2 = np.array(
            [-0.003912773485618989, 0.29969367235921324, 1.4896964934919243],
            dtype=np.double,
        )
        pos_quad_3 = np.array(
            [0.8063607699386893, -0.00040573095469479846, 1.4825609159860986],
            dtype=np.double,
        )

        q1_eq = self.normalize(pos_0 - pos_quad_1)
        q2_eq = self.normalize(pos_0 - pos_quad_2)
        q3_eq = self.normalize(pos_0 - pos_quad_3)
        q_eq_list = [q1_eq, q2_eq, q3_eq]

        _, tensions_eq = self.build_hover_equilibrium_lambdas(
            payload_mass=self.mass,
            gravity=self.gravity,
            q_eq_list=q_eq_list,
        )

        self.tension_min = 0.2 * np.asarray(tensions_eq, dtype=np.double)
        self.tension_max = 10.0 * np.asarray(tensions_eq, dtype=np.double)

        self.n_init = np.hstack(q_eq_list)
        self.r_init = np.zeros((self.robot_num * 3,), dtype=np.double)
        self.x_0 = np.hstack((pos_0, vel_0, self.n_init, self.r_init))
        self.u_equilibrium = np.zeros((self.robot_num * 3,), dtype=np.double)

        self.acceleration_limit = np.array(
            [20.0, 20.0, 20.0] * self.robot_num,
            dtype=np.double,
        )
        self.u_min = -self.acceleration_limit.copy()
        self.u_max = self.acceleration_limit.copy()

        self.project_root = Path(__file__).resolve().parents[1]
        self.code_export_directory = self.project_root / "c_generated_code"
        self.json_file = self.project_root / "acados_ocp_planner_payload_pointmass_multiple.json"

        self.ocp = self.solver(self.x_0)
        AcadosOcpSolver(
            self.ocp,
            json_file=str(self.json_file),
            build=True,
            generate=True,
        )

    @staticmethod
    def normalize(v: np.ndarray) -> np.ndarray:
        n = np.linalg.norm(v)
        if n < 1e-9:
            raise ValueError("Cannot normalize a near-zero vector.")
        return v / n

    def build_hover_equilibrium_lambdas(
        self,
        payload_mass: float,
        gravity: float,
        q_eq_list: List[np.ndarray],
    ):
        N = np.column_stack(q_eq_list)
        rhs = -payload_mass * gravity * np.array(self.e3, dtype=np.double).reshape((3,))
        tensions = np.linalg.solve(N, rhs)
        lambdas_eq = [tensions[i] * q_eq_list[i] for i in range(3)]
        return lambdas_eq, tensions

    def payload_model(self):
        model_name = "planner_payload_pointmass_multiple"

        x_p = ca.MX.sym("x_p", 3, 1)
        v_p = ca.MX.sym("v_p", 3, 1)
        n1 = ca.MX.sym("n1", 3, 1)
        n2 = ca.MX.sym("n2", 3, 1)
        n3 = ca.MX.sym("n3", 3, 1)
        r1 = ca.MX.sym("r1", 3, 1)
        r2 = ca.MX.sym("r2", 3, 1)
        r3 = ca.MX.sym("r3", 3, 1)

        x = ca.vertcat(x_p, v_p, n1, n2, n3, r1, r2, r3)

        a_1 = ca.MX.sym("a_1", 3, 1)
        a_2 = ca.MX.sym("a_2", 3, 1)
        a_3 = ca.MX.sym("a_3", 3, 1)
        u = ca.vertcat(a_1, a_2, a_3)

        N = ca.hcat([n1, n2, n3])
        W = ca.hcat([r1, r2, r3])
        U = ca.hcat([a_1, a_2, a_3])

        d_1 = ca.dot(N[:, 0], U[:, 0]) - self.length * ca.dot(W[:, 0], W[:, 0])
        d_2 = ca.dot(N[:, 1], U[:, 1]) - self.length * ca.dot(W[:, 1], W[:, 1])
        d_3 = ca.dot(N[:, 2], U[:, 2]) - self.length * ca.dot(W[:, 2], W[:, 2])

        I3 = ca.MX.eye(3)
        z = ca.MX.zeros(1, 1)
        M = ca.vertcat(
            ca.hcat([self.mass * I3, n1, n2, n3]),
            ca.hcat([n1.T, z, z, z]),
            ca.hcat([n2.T, z, z, z]),
            ca.hcat([n3.T, z, z, z]),
        )

        b = ca.vertcat(-self.mass * self.gravity * self.e3, d_1, d_2, d_3)
        acceleration_tension = ca.solve(M, b)
        a_p = acceleration_tension[0:3]
        tensions_expression = acceleration_tension[3:6]

        n1_dot = ca.cross(r1, n1)
        n2_dot = ca.cross(r2, n2)
        n3_dot = ca.cross(r3, n3)
        r1_dot = (1.0 / self.length) * ca.cross(n1, (a_p - U[:, 0]))
        r2_dot = (1.0 / self.length) * ca.cross(n2, (a_p - U[:, 1]))
        r3_dot = (1.0 / self.length) * ca.cross(n3, (a_p - U[:, 2]))

        f_expl = ca.vertcat(v_p, a_p, n1_dot, n2_dot, n3_dot, r1_dot, r2_dot, r3_dot)

        nx = x.shape[0]
        x_dot = ca.MX.sym("x_dot", nx, 1)
        model = AcadosModel()
        model.x = x
        model.xdot = x_dot
        model.x_dot = x_dot
        model.u = u
        model.f_expl_expr = f_expl
        model.f_impl_expr = x_dot - f_expl
        model.p = ca.MX.sym("ref_params", nx + u.shape[0], 1)
        model.name = model_name
        return model, tensions_expression

    def solver(self, x0):
        model, tensions_expression = self.payload_model()

        ocp = AcadosOcp()
        ocp.model = model
        ocp.code_gen_opts.code_export_directory = str(self.code_export_directory)

        nx = model.x.size()[0]
        nu = model.u.size()[0]
        ocp.dims.N = self.N_prediction
        ocp.cost.cost_type = "EXTERNAL"
        ocp.cost.cost_type_e = "EXTERNAL"

        x = ocp.model.x
        u = ocp.model.u
        p = ocp.model.p

        x_p = x[0:3]
        v_p = x[3:6]
        n1 = x[6:9]
        n2 = x[9:12]
        n3 = x[12:15]
        r1 = x[15:18]
        r2 = x[18:21]
        r3 = x[21:24]
        a_q1 = u[0:3]
        a_q2 = u[3:6]
        a_q3 = u[6:9]

        x_p_d = p[0:3]
        v_p_d = p[3:6]
        n1_d = p[6:9]
        n2_d = p[9:12]
        n3_d = p[12:15]
        r1_d = p[15:18]
        r2_d = p[18:21]
        r3_d = p[21:24]

        error_position = x_p - x_p_d
        error_velocity = v_p - v_p_d
        error_n1 = ca.cross(n1_d, n1)
        error_n2 = ca.cross(n2_d, n2)
        error_n3 = ca.cross(n3_d, n3)
        r1_error = r1 - (ca.MX.eye(3) - n1 @ n1.T) @ r1_d
        r2_error = r2 - (ca.MX.eye(3) - n2 @ n2.T) @ r2_d
        r3_error = r3 - (ca.MX.eye(3) - n3 @ n3.T) @ r3_d

        Kp = ca.DM(self.Kp)
        Kv = ca.DM(self.Kv)
        Kp_n = ca.DM(self.Kp_n)
        Kp_r = ca.DM(self.Kp_r)
        R_q = ca.DM(self.R_q)

        lyapunov_position = (
            error_position.T @ Kp @ error_position
            + self.mass * (error_velocity.T @ Kv @ error_velocity)
        )

        ocp.model.cost_expr_ext_cost = (
            lyapunov_position
            + error_n1.T @ Kp_n @ error_n1
            + error_n2.T @ Kp_n @ error_n2
            + error_n3.T @ Kp_n @ error_n3
            + r1_error.T @ Kp_r @ r1_error
            + r2_error.T @ Kp_r @ r2_error
            + r3_error.T @ Kp_r @ r3_error
            + a_q1.T @ R_q @ a_q1
            + a_q2.T @ R_q @ a_q2
            + a_q3.T @ R_q @ a_q3
        )
        ocp.model.cost_expr_ext_cost_e = (
            lyapunov_position
            + error_n1.T @ Kp_n @ error_n1
            + error_n2.T @ Kp_n @ error_n2
            + error_n3.T @ Kp_n @ error_n3
            + r1_error.T @ Kp_r @ r1_error
            + r2_error.T @ Kp_r @ r2_error
            + r3_error.T @ Kp_r @ r3_error
        )

        ref_params = np.hstack((self.x_0, self.u_equilibrium))
        ocp.parameter_values = ref_params

        ocp.constraints.constr_type = "BGH"
        ocp.constraints.lbu = self.u_min
        ocp.constraints.ubu = self.u_max
        ocp.constraints.idxbu = np.arange(nu, dtype=np.int32)
        ocp.constraints.x0 = x0

        ocp.model.con_h_expr = ca.vertcat(
            ca.dot(n1, n1),
            ca.dot(n2, n2),
            ca.dot(n3, n3),
            tensions_expression,
        )
        nh = 6
        nsh = nh
        ocp.cost.zl = self.norm_constraint_slack_weight * np.ones((nsh,))
        ocp.cost.Zl = self.norm_constraint_slack_weight * np.ones((nsh,))
        ocp.cost.zu = self.norm_constraint_slack_weight * np.ones((nsh,))
        ocp.cost.Zu = self.norm_constraint_slack_weight * np.ones((nsh,))
        ocp.constraints.lh = np.concatenate(
            (
                np.full((3,), 1.0 - self.unit_vector_norm_tol, dtype=np.double),
                self.tension_min.reshape((3,)),
            )
        )
        ocp.constraints.uh = np.concatenate(
            (
                np.full((3,), 1.0 + self.unit_vector_norm_tol, dtype=np.double),
                self.tension_max.reshape((3,)),
            )
        )
        ocp.constraints.lsh = np.zeros((nsh,))
        ocp.constraints.ush = np.zeros((nsh,))
        ocp.constraints.idxsh = np.arange(nsh, dtype=np.int32)

        ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
        ocp.solver_options.qp_solver_cond_N = self.N_prediction
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "IRK"
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 2
        ocp.solver_options.sim_method_newton_iter = 20
        ocp.solver_options.sim_method_newton_tol = 1e-10
        ocp.solver_options.levenberg_marquardt = 10.0
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.nlp_solver_max_iter = 1
        ocp.solver_options.Tsim = self.ts
        ocp.solver_options.tf = self.t_N
        ocp.solver_options.N_horizon = self.N_prediction
        ocp.solver_options.regularize_method = "CONVEXIFY"
        return ocp


def main(params):
    PayloadPointMassMultipleBuilder(params)
    return None


def cli():
    if len(sys.argv) < 2:
        raise SystemExit("usage: build_payload_planner_pointmass_multiple.py <config.yaml>")
    path_to_yaml = os.path.abspath(sys.argv[1])
    params = yaml_to_dict(path_to_yaml)
    print(params)
    main(params)


if __name__ == "__main__":
    cli()
