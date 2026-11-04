#!/usr/bin/env python3
"""Detect execution-level stalls: cmd says move, odometry says it didn't.

Motivation: the campus run on 2026-11-04 commanded vx=0.5 m/s for ~150 s
while the robot moved 0.0 m (hillside).  Nothing in the stack noticed.

Two detectors:
  stuck       |cmd_vx| high but net displacement ~0 over a window
  spin_stuck  |cmd_wz| high and it turned, but never translated

On every event it logs the context needed to tell the three causes apart:
  slope      -> z / roll / pitch of the odometry
  obstacle   -> nearest occupied cell in a forward cone of /globalMap
  policy slip-> flat, no obstacle ahead, yet zero displacement
It also publishes std_msgs/Bool on ~stuck_topic (for a future recovery node).

Log-only: it never publishes commands.
"""

import math
from collections import deque

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rpy_from_quat(x, y, z, w):
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    return roll, pitch


class StuckDetector:
    def __init__(self):
        self.cmd_thr = float(rospy.get_param("~cmd_thr", 0.15))
        self.spin_cmd_thr = float(rospy.get_param("~spin_cmd_thr", 0.30))
        self.window = float(rospy.get_param("~window", 5.0))
        self.disp_thr = float(rospy.get_param("~disp_thr", 0.10))
        self.yaw_turn_thr = float(rospy.get_param("~yaw_turn_thr", 6.28))
        self.startup_ignore = float(rospy.get_param("~startup_ignore", 20.0))
        self.event_period = float(rospy.get_param("~event_period", 10.0))
        self.stuck_topic = rospy.get_param("~stuck_topic", "/stuck")
        self.cone_deg = float(rospy.get_param("~obstacle_cone_deg", 40.0))
        self.cone_range = float(rospy.get_param("~obstacle_range", 1.6))

        self.pose_hist = deque(maxlen=4000)   # (t, x, y, yaw, z, roll, pitch)
        self.cmd_hist = deque(maxlen=4000)    # (t, vx, wz)
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.cmd_t = None
        self.map = None
        self.map_res = 0.3
        self.map_ox = 0.0
        self.map_oy = 0.0
        self.last_event_t = -1e9
        self.stuck = False

        self.pub = rospy.Publisher(self.stuck_topic, Bool, queue_size=1)
        rospy.Subscriber("/planning/cmd_vel_raw", Twist, self.cmd_cb, queue_size=10)
        rospy.Subscriber("/ground_truth/state", Odometry, self.odom_cb, queue_size=20)
        rospy.Subscriber("/globalMap", OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.25), self.check)
        rospy.loginfo(
            "stuck_detector: cmd_thr=%.2f window=%.1fs disp_thr=%.2fm",
            self.cmd_thr, self.window, self.disp_thr)

    def cmd_cb(self, msg):
        self.cmd_vx = msg.linear.x
        self.cmd_wz = msg.angular.z
        self.cmd_t = rospy.Time.now().to_sec()
        self.cmd_hist.append((self.cmd_t, msg.linear.x, msg.angular.z))

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll, pitch = rpy_from_quat(q.x, q.y, q.z, q.w)
        self.pose_hist.append((msg.header.stamp.to_sec(), p.x, p.y,
                               yaw_from_quat(q.x, q.y, q.z, q.w),
                               p.z, roll, pitch))

    def map_cb(self, msg):
        self.map = msg
        self.map_res = msg.info.resolution
        self.map_ox = msg.info.origin.position.x
        self.map_oy = msg.info.origin.position.y

    def obstacle_ahead(self, x, y, yaw):
        """Nearest occupied cell (>=50) within a forward cone, in metres."""
        if self.map is None:
            return None
        data = self.map.data
        w = self.map.info.width
        h = self.map.info.height
        best = None
        r = int(self.cone_range / self.map_res) + 1
        ix = int((x - self.map_ox) / self.map_res)
        iy = int((y - self.map_oy) / self.map_res)
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                gx, gy = ix + dx, iy + dy
                if gx < 0 or gy < 0 or gx >= w or gy >= h:
                    continue
                if data[gx + gy * w] < 50:
                    continue
                cx = (gx + 0.5) * self.map_res + self.map_ox
                cy = (gy + 0.5) * self.map_res + self.map_oy
                d = math.hypot(cx - x, cy - y)
                if d > self.cone_range:
                    continue
                ang = math.atan2(cy - y, cx - x) - yaw
                while ang > math.pi:
                    ang -= 2 * math.pi
                while ang < -math.pi:
                    ang += 2 * math.pi
                if abs(ang) <= math.radians(self.cone_deg):
                    best = d if best is None else min(best, d)
        return best

    def check(self, _event):
        if len(self.pose_hist) < 10:
            return
        t_now = self.pose_hist[-1][0]
        if t_now < self.startup_ignore:
            return
        t_start = t_now - self.window
        window = [p for p in self.pose_hist if p[0] >= t_start]
        if len(window) < 5 or window[0][0] > t_start + self.window * 0.5:
            return
        xs = np.array([p[1] for p in window])
        ys = np.array([p[2] for p in window])
        yaws = np.array([p[3] for p in window])
        disp = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        dyaw = abs(math.atan2(math.sin(yaws[-1] - yaws[0]),
                              math.cos(yaws[-1] - yaws[0])))
        # Sustained command: average |vx| over the same time window.  The
        # instantaneous value flags the alignment turn right after
        # pathFollower starts (vx briefly 0.16 with wz large).
        cmds = [c for c in self.cmd_hist if c[0] >= t_start]
        cmd_vx = float(np.mean([abs(c[1]) for c in cmds])) if cmds else abs(self.cmd_vx)
        cmd_wz = float(np.mean([abs(c[2]) for c in cmds])) if cmds else abs(self.cmd_wz)

        stuck = cmd_vx > self.cmd_thr and disp < self.disp_thr
        spin_stuck = (not stuck and cmd_wz > self.spin_cmd_thr and
                      dyaw > self.yaw_turn_thr and disp < self.disp_thr)
        self.stuck = bool(stuck or spin_stuck)
        self.pub.publish(Bool(data=self.stuck))
        if not self.stuck:
            return
        if t_now - self.last_event_t < self.event_period:
            return
        self.last_event_t = t_now

        x, y, z = xs[-1], ys[-1], window[-1][4]
        roll, pitch = window[-1][5], window[-1][6]
        obs = self.obstacle_ahead(x, y, yaws[-1])
        obs_txt = "none" if obs is None else "%.2f m" % obs
        kind = "spin_stuck" if spin_stuck else "stuck"
        rospy.logwarn(
            "[%s] t=%.1f pose=(%.1f,%.1f) z=%.2f roll=%.1f pitch=%.1f deg "
            "cmd=(vx %.2f, wz %.2f) disp=%.2fm dyaw=%.2frad in %.1fs "
            "obstacle_ahead=%s",
            kind, t_now, x, y, z, math.degrees(roll), math.degrees(pitch),
            self.cmd_vx, self.cmd_wz, disp, dyaw, self.window, obs_txt)


def main():
    rospy.init_node("stuck_detector")
    StuckDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
