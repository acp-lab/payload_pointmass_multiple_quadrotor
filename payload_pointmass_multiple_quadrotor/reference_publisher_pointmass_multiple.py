#!/usr/bin/env python3
from __future__ import annotations

import time

import numpy as np
import rclpy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from payload_pointmass_multiple_quadrotor.lim_min_multiple_simple import (
    plan_three_quad_point_mass,
)
from payload_pointmass_multiple_quadrotor.lissajous_multiple_simple import (
    plan_three_quad_lissajous_payload,
)


class PointMassMultipleReferencePublisher(Node):
    def __init__(self):
        super().__init__("pointmass_multiple_reference_publisher")

        self.robot_num = 3
        self.n_x = 24
        self.n_u = 9

        self.t_N = 1.5
        self.N_prediction = 31
        self.ts = self.t_N / self.N_prediction
        self.planner_duration = 5.0
        self.planner_type = "lissajous"

        self.length = 1.0
        self.mass = 0.2
        self.gravity = 9.81

        self.payload_init = np.array(
            [0.34049933598831467, -0.0007520805616463245, 0.8936953489145677],
            dtype=np.double,
        )
        self.planner_goal = np.array([5.0, 1.2, 0.3], dtype=np.double)

        self.lissajous_offset = self.payload_init.copy()
        self.lissajous_x_amp = 0.5
        self.lissajous_y_amp = 0.5
        self.lissajous_z_amp = 0.2
        self.lissajous_period = 4.0
        self.lissajous_num_cycles = 1.0
        self.lissajous_ramp_time = 1.0
        self.lissajous_x_num_periods = 1.0
        self.lissajous_y_num_periods = 2.0
        self.lissajous_z_num_periods = 1.0

        self.reference_plan = self.build_reference_plan()
        self.reference_start_time = None

        self.publisher_reference_states_flat = self.create_publisher(
            Float64MultiArray,
            "payload_reference_states_flat",
            10,
        )
        self.publisher_reference_inputs_flat = self.create_publisher(
            Float64MultiArray,
            "payload_reference_inputs_flat",
            10,
        )
        self.publisher_reference_path = self.create_publisher(
            Path,
            "payload_reference_path_pointmass_multiple",
            10,
        )

        self.timer = self.create_timer(self.ts, self.run)

    def build_reference_plan(self):
        common_kwargs = {
            "n_samples": max(self.N_prediction + 1, 201),
            "payload_mass": self.mass,
            "gravity": self.gravity,
            "cable_lengths": np.full((self.robot_num,), self.length, dtype=np.double),
        }

        if self.planner_type == "lissajous":
            return plan_three_quad_lissajous_payload(
                offset=np.asarray(self.lissajous_offset, dtype=np.double),
                x_amp=self.lissajous_x_amp,
                y_amp=self.lissajous_y_amp,
                z_amp=self.lissajous_z_amp,
                period=self.lissajous_period,
                num_cycles=self.lissajous_num_cycles,
                ramp_time=self.lissajous_ramp_time,
                x_num_periods=self.lissajous_x_num_periods,
                y_num_periods=self.lissajous_y_num_periods,
                z_num_periods=self.lissajous_z_num_periods,
                **common_kwargs,
            )

        return plan_three_quad_point_mass(
            p0=self.payload_init,
            pf=self.planner_goal,
            T_total=self.planner_duration,
            **common_kwargs,
        )

    def reference_from_plan(self, t_query: float):
        times = self.reference_plan["t"]
        idx = int(
            np.clip(np.searchsorted(times, t_query, side="left"), 0, len(times) - 1)
        )

        xd = np.zeros((self.n_x,), dtype=np.double)
        ud = np.zeros((self.n_u,), dtype=np.double)

        xd[0:3] = self.reference_plan["payload_p"][idx]
        xd[3:6] = self.reference_plan["payload_v"][idx]

        q_ref = self.reference_plan["q"][:, idx, :]
        qdot_ref = self.reference_plan["qdot"][:, idx, :]
        r_ref = np.cross(q_ref, qdot_ref)

        xd[6:15] = q_ref.reshape((self.robot_num * 3,))
        xd[15:24] = r_ref.reshape((self.robot_num * 3,))
        ud[:] = self.reference_plan["quad_a"][:, idx, :].reshape((self.robot_num * 3,))
        return xd, ud

    def publish_reference_path(self, elapsed: float):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "world"

        plan_end_time = float(self.reference_plan["t"][-1])
        for stage in range(self.N_prediction):
            t_stage = min(elapsed + stage * self.ts, plan_end_time)
            yref, _ = self.reference_from_plan(t_stage)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = yref[0]
            pose.pose.position.y = yref[1]
            pose.pose.position.z = yref[2]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.publisher_reference_path.publish(path_msg)

    def publish_reference_horizon(self, elapsed: float):
        reference_states = np.zeros((self.n_x, self.N_prediction), dtype=np.double)
        reference_inputs = np.zeros((self.n_u, self.N_prediction), dtype=np.double)

        plan_end_time = float(self.reference_plan["t"][-1])
        for stage in range(self.N_prediction):
            t_stage = min(elapsed + stage * self.ts, plan_end_time)
            yref, uref = self.reference_from_plan(t_stage)
            reference_states[:, stage] = yref
            reference_inputs[:, stage] = uref

        states_msg = Float64MultiArray()
        states_msg.data = reference_states.reshape((-1,), order="F").tolist()
        self.publisher_reference_states_flat.publish(states_msg)

        inputs_msg = Float64MultiArray()
        inputs_msg.data = reference_inputs.reshape((-1,), order="F").tolist()
        self.publisher_reference_inputs_flat.publish(inputs_msg)

        self.publish_reference_path(elapsed)

    def run(self):
        if self.reference_start_time is None:
            self.reference_start_time = time.monotonic()

        current_time = time.monotonic()
        plan_end_time = float(self.reference_plan["t"][-1])
        elapsed = min(current_time - self.reference_start_time, plan_end_time)
        self.publish_reference_horizon(elapsed)


def main(args=None):
    rclpy.init(args=args)
    node = PointMassMultipleReferencePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
