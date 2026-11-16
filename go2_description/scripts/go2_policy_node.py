#!/usr/bin/env python3
"""ROS 1 bridge for the exported go2_rl_lab TorchScript policy.

This is deliberately a small first-port, not a full copy of the ROS 2
supervisor: it assumes the robot is spawned in the standing pose and starts
the 50 Hz policy after joint states and IMU are fresh.  A 500 Hz PD loop owns
the effort commands so policy inference never blocks the actuator loop.
"""

import math
import threading
import time

import numpy as np
import rospy
import torch
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

JOINT_NAMES = (
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
)
DEFAULT_DOF_POS = np.array(
    [0.1, 0.8, -1.5, -0.1, 0.8, -1.5,
     0.1, 1.0, -1.5, -0.1, 1.0, -1.5], dtype=np.float64)
# URDF limits, leg-major.  The policy target is clipped before PD.
JOINT_LOWER = np.array(
    [-1.0472, -1.5708, -2.7227,
     -1.0472, -1.5708, -2.7227,
     -1.0472, -1.5708, -2.7227,
     -1.0472, -1.5708, -2.7227], dtype=np.float64)
JOINT_UPPER = np.array(
    [1.0472, 3.4907, -0.83776,
     1.0472, 3.4907, -0.83776,
     1.0472, 3.4907, -0.83776,
     1.0472, 3.4907, -0.83776], dtype=np.float64)
TORQUE_LIMITS = np.array(
    [23.7, 23.7, 35.55,
     23.7, 23.7, 35.55,
     23.7, 23.7, 35.55,
     23.7, 23.7, 35.55], dtype=np.float64)
GRAVITY_WORLD = np.array([0.0, 0.0, -1.0], dtype=np.float64)


def projected_gravity(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid IMU quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)
    return rotation.T @ GRAVITY_WORLD


class Go2PolicyNode:
    def __init__(self):
        self.model_path = rospy.get_param("~model_path")
        self.kp = float(rospy.get_param("~kp", 25.0))
        self.kd = float(rospy.get_param("~kd", 0.5))
        self.action_scale = float(rospy.get_param("~action_scale", 0.25))
        self.policy_rate = float(rospy.get_param("~policy_rate", 50.0))
        self.control_rate = float(rospy.get_param("~control_rate", 500.0))
        self.command_scale = np.asarray(
            rospy.get_param("~command_scale", [0.5, 1.0, 1.0]),
            dtype=np.float64)
        self.command_limits = np.asarray(
            rospy.get_param("~command_limits", [2.0, 1.0, 3.0]),
            dtype=np.float64)
        self.slew_rates = np.asarray(
            rospy.get_param("~slew_rates", [0.5, 0.4, 1.0]),
            dtype=np.float64)
        self.torch_threads = int(rospy.get_param("~torch_threads", 1))
        self.fresh_timeout = float(rospy.get_param("~fresh_timeout", 0.5))
        self.policy_dt = 1.0 / self.policy_rate
        self.control_dt = 1.0 / self.control_rate

        torch.set_num_threads(max(1, self.torch_threads))
        try:
            torch.set_num_interop_threads(1)
        except RuntimeError:
            pass
        self.policy = torch.jit.load(
            self.model_path, map_location="cpu")
        self.policy.eval()
        self.policy.reset()
        rospy.loginfo(
            "go2_rl_lab loaded path=%s kp=%.1f kd=%.1f rate=%.0fHz "
            "command_scale=%s limits=%s torch_threads=%d",
            self.model_path, self.kp, self.kd, self.policy_rate,
            self.command_scale.tolist(), self.command_limits.tolist(),
            self.torch_threads)

        self.lock = threading.Lock()
        self.joint_index = {name: i for i, name in enumerate(JOINT_NAMES)}
        self.q = DEFAULT_DOF_POS.copy()
        self.dq = np.zeros(12, dtype=np.float64)
        self.seen_joints = np.zeros(12, dtype=bool)
        self.have_joint = False
        self.have_imu = False
        self.last_joint_ros = None
        self.last_imu_ros = None
        self.gravity_body = GRAVITY_WORLD.copy()
        self.base_ang_vel = np.zeros(3, dtype=np.float64)
        self.command = np.zeros(3, dtype=np.float64)
        self.filtered_command = np.zeros(3, dtype=np.float64)
        self.last_action = np.zeros(12, dtype=np.float32)
        self.target = DEFAULT_DOF_POS.copy()
        self.handoff = 0.0
        self.handoff_duration = 0.30
        self.ready_since = None
        self.last_policy_log = 0.0
        self.policy_steps = 0
        self.inference_ms = 0.0

        self.effort_pub = rospy.Publisher(
            "/joint_group_effort_controller/command",
            Float64MultiArray, queue_size=1)
        rospy.Subscriber(
            "/joint_states", JointState, self.joint_callback, queue_size=50)
        rospy.Subscriber(
            "/go2/imu", Imu, self.imu_callback, queue_size=50)
        rospy.Subscriber(
            "/planning/cmd_vel_raw", Twist, self.command_callback, queue_size=10)
        self.policy_thread = threading.Thread(
            target=self.policy_loop, name="go2_policy_50hz", daemon=True)
        self.policy_thread.start()
        self.control_timer = rospy.Timer(
            rospy.Duration(self.control_dt), self.control_callback)
        rospy.loginfo("go2 policy node waiting for /joint_states and /go2/imu")

    def joint_callback(self, msg):
        if not msg.name:
            return
        stamp = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()
        with self.lock:
            for i, name in enumerate(msg.name):
                index = self.joint_index.get(name)
                if index is None or i >= len(msg.position):
                    continue
                self.q[index] = float(msg.position[i])
                if i < len(msg.velocity):
                    self.dq[index] = float(msg.velocity[i])
                self.seen_joints[index] = True
            self.have_joint = bool(self.seen_joints.all())
            if self.have_joint:
                self.last_joint_ros = stamp

    def imu_callback(self, msg):
        q = msg.orientation
        try:
            gravity = projected_gravity(q.x, q.y, q.z, q.w)
        except ValueError:
            self.have_imu = False
            return
        with self.lock:
            self.gravity_body = gravity
            self.base_ang_vel = np.array(
                [msg.angular_velocity.x, msg.angular_velocity.y,
                 msg.angular_velocity.z], dtype=np.float64)
            self.have_imu = True
            self.last_imu_ros = (
                msg.header.stamp if not msg.header.stamp.is_zero()
                else rospy.Time.now())

    def command_callback(self, msg):
        command = np.array(
            [msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float64)
        command = np.clip(command, -self.command_limits, self.command_limits)
        with self.lock:
            self.command = command

    def sensors_fresh(self, now):
        if not (self.have_joint and self.have_imu):
            return False
        if self.last_joint_ros is None or self.last_imu_ros is None:
            return False
        age_joint = (now - self.last_joint_ros).to_sec()
        age_imu = (now - self.last_imu_ros).to_sec()
        return age_joint <= self.fresh_timeout and age_imu <= self.fresh_timeout

    def policy_loop(self):
        next_tick = time.monotonic()
        while not rospy.is_shutdown():
            next_tick += self.policy_dt
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
            now = rospy.Time.now()
            if not self.sensors_fresh(now):
                continue
            if self.ready_since is None:
                self.ready_since = now
                rospy.loginfo("joints and IMU fresh; holding stance before policy")
            if (now - self.ready_since).to_sec() < 1.0:
                continue

            with self.lock:
                command = self.command.copy()
                filtered = self.filtered_command.copy()
                q = self.q.copy()
                dq = self.dq.copy()
                last_action = self.last_action.copy()
            delta = np.clip(
                command - filtered,
                -self.slew_rates * self.policy_dt,
                self.slew_rates * self.policy_dt)
            filtered = filtered + delta
            obs = np.zeros(45, dtype=np.float32)
            obs[0:3] = self.base_ang_vel * 0.25
            obs[3:6] = self.gravity_body
            obs[6:9] = filtered * self.command_scale
            obs[9:21] = q - DEFAULT_DOF_POS
            obs[21:33] = dq * 0.05
            obs[33:45] = last_action
            obs = np.clip(obs, -100.0, 100.0)
            started = time.monotonic()
            try:
                with torch.inference_mode():
                    action = self.policy(
                        torch.from_numpy(obs.reshape(1, 45))).detach().numpy()
                action = np.asarray(action, dtype=np.float32).reshape(-1)
                if action.shape != (12,) or not np.isfinite(action).all():
                    raise ValueError("invalid policy action")
                action = np.clip(action, -100.0, 100.0)
                target = DEFAULT_DOF_POS + self.action_scale * action
                target = np.clip(target, JOINT_LOWER, JOINT_UPPER)
            except Exception as exc:
                rospy.logerr_throttle(2.0, "policy inference failed: %s", exc)
                continue
            elapsed_ms = (time.monotonic() - started) * 1000.0
            with self.lock:
                self.filtered_command = filtered
                self.last_action = action.astype(np.float32)
                if self.handoff < self.handoff_duration:
                    self.handoff = min(
                        self.handoff_duration, self.handoff + self.policy_dt)
                    progress = self.handoff / self.handoff_duration
                    self.target = DEFAULT_DOF_POS + progress * (
                        target - DEFAULT_DOF_POS)
                else:
                    self.target = target
                self.policy_steps += 1
                self.inference_ms = elapsed_ms
            if time.monotonic() - self.last_policy_log > 5.0:
                self.last_policy_log = time.monotonic()
                rospy.loginfo(
                    "policy steps=%d cmd=[%.2f %.2f %.2f] "
                    "filtered=[%.2f %.2f %.2f] inference=%.1fms",
                    self.policy_steps, command[0], command[1], command[2],
                    filtered[0], filtered[1], filtered[2], elapsed_ms)

    def control_callback(self, _event):
        with self.lock:
            target = self.target.copy()
            q = self.q.copy()
            dq = self.dq.copy()
            have_joint = self.have_joint
        if not have_joint:
            return
        effort = self.kp * (target - q) - self.kd * dq
        effort = np.clip(effort, -TORQUE_LIMITS, TORQUE_LIMITS)
        msg = Float64MultiArray()
        msg.data = effort.astype(float).tolist()
        self.effort_pub.publish(msg)


def main():
    rospy.init_node("go2_policy_node")
    Go2PolicyNode()
    rospy.spin()


if __name__ == "__main__":
    main()
