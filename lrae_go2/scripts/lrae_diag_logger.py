#!/usr/bin/env python3
"""Diagnostic CSV logger for LRAE + Go2 NewMine runs.

Writes rows at 50 Hz from the latest robot pose/twist, localPlanner /path
geometry, look_ahead_goal geometry and /planning/cmd_vel_raw.  The command is
zeroed if no cmd_vel has arrived for 0.2 s, so a silent/stopped planner still
produces a complete diag.csv instead of a header-only file.  The /path columns replicate pathFollower's
look-ahead walk (advance while dis < lookAheadDis, then measure the active
segment) so the dirDiff speed gate can be checked offline.  Also samples
/globalMap (0 free, 100 occupied, -1 unknown, 100 border) every
--coverage-period seconds.

Run inside the container:
  python3 lrae_diag_logger.py --output /out/diag.csv
"""

import argparse
import csv
import math
import threading

import rospy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, Path, OccupancyGrid

LOOK_AHEAD_DIS = 0.5  # pathFollower default lookAheadDis
STOP_DIS_THRE = 0.2


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class DiagLogger:
    def __init__(self, out_path, coverage_period, odom_topic):
        self.lock = threading.Lock()
        # latest odom
        self.t_ros = None
        self.x = None
        self.y = None
        self.yaw = None
        self.twist_vx = None
        self.twist_wz = None
        # latest /path (localPlanner) geometry, robot frame at receipt
        self.path_size = 0
        self.path_end_dis = None
        self.path_heading_err = None
        # latest look_ahead_goal
        self.goal_x = None
        self.goal_y = None
        self.goal_dist = None
        self.goal_heading_err = None
        # latest global exploration path
        self.expath_size = 0
        self.expath_end_dis = None
        # coverage
        self.cov_rows = []
        self.msg_count = 0
        # latest command; a timer writes rows even when pathFollower is silent
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.last_cmd_wall = None

        self.f = open(out_path, "w", newline="")
        self.writer = csv.writer(self.f)
        self.writer.writerow([
            "t_wall", "t_ros", "x", "y", "yaw",
            "cmd_vx", "cmd_wz",
            "twist_vx", "twist_wz",
            "path_size", "path_end_dis", "path_heading_err",
            "goal_dist", "goal_heading_err",
            "expath_size", "expath_end_dis",
        ])
        self.cov_writer = None
        self.cov_f = None
        if coverage_period > 0:
            self.cov_path = out_path.replace(".csv", "_coverage.csv")
            self.cov_f = open(self.cov_path, "w", newline="")
            self.cov_writer = csv.writer(self.cov_f)
            self.cov_writer.writerow(["t_ros", "unknown", "free", "occupied"])

        rospy.Subscriber(odom_topic, Odometry,
                         self.odom_cb, queue_size=5)
        rospy.Subscriber("/planning/cmd_vel_raw", Twist,
                         self.cmd_cb, queue_size=50)
        rospy.Subscriber("/path", Path, self.path_cb, queue_size=5)
        rospy.Subscriber("/look_ahead_goal", PointStamped,
                         self.goal_cb, queue_size=5)
        rospy.Subscriber("/exporation_path", Path,
                         self.expath_cb, queue_size=10)
        rospy.Subscriber("/globalMap", OccupancyGrid,
                         self.map_cb, queue_size=1)
        rospy.Timer(rospy.Duration(5.0), self.flush_cb)
        # 50 Hz row writer: a silent/stopped planner still contributes rows
        # with cmd_vx=cmd_wz=0 instead of producing a header-only dump.
        rospy.Timer(rospy.Duration(0.02), self.timer_cb)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self.lock:
            self.t_ros = msg.header.stamp.to_sec()
            self.x = p.x
            self.y = p.y
            self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
            self.twist_vx = msg.twist.twist.linear.x
            self.twist_wz = msg.twist.twist.angular.z

    def _path_geometry(self, msg, x, y, yaw):
        """Replicate pathFollower's look-ahead walk at path receipt time."""
        n = len(msg.poses)
        end_dis = None
        heading_err = None
        if n >= 1 and x is not None:
            ex = msg.poses[n - 1].pose.position.x - x
            ey = msg.poses[n - 1].pose.position.y - y
            end_dis = math.hypot(ex, ey)
            idx = 0
            while idx < n - 1:
                dx = msg.poses[idx].pose.position.x - x
                dy = msg.poses[idx].pose.position.y - y
                if math.hypot(dx, dy) < LOOK_AHEAD_DIS:
                    idx += 1
                else:
                    break
            dx = msg.poses[idx].pose.position.x - x
            dy = msg.poses[idx].pose.position.y - y
            if math.hypot(dx, dy) > 1e-6:
                heading_err = wrap(math.atan2(dy, dx) - yaw)
        return n, end_dis, heading_err

    def path_cb(self, msg):
        with self.lock:
            # localPlanner publishes /path in the "vehicle" frame at receipt.
            # Using the global pose here adds the map offset (campus start is
            # far from the origin) and reports a bogus 57 m path / heading.
            if msg.header.frame_id in ("vehicle", "base", "base_link"):
                x0, y0, yaw0 = 0.0, 0.0, 0.0
            else:
                x0, y0, yaw0 = self.x, self.y, self.yaw
            n, end_dis, heading_err = self._path_geometry(
                msg, x0, y0, yaw0)
            self.path_size = n
            self.path_end_dis = end_dis
            self.path_heading_err = heading_err

    def goal_cb(self, msg):
        with self.lock:
            self.goal_x = msg.point.x
            self.goal_y = msg.point.y
            if self.x is not None:
                dx = self.goal_x - self.x
                dy = self.goal_y - self.y
                self.goal_dist = math.hypot(dx, dy)
                self.goal_heading_err = wrap(math.atan2(dy, dx) - self.yaw)

    def expath_cb(self, msg):
        with self.lock:
            self.expath_size = len(msg.poses)
            if self.expath_size and self.x is not None:
                ex = msg.poses[-1].pose.position.x - self.x
                ey = msg.poses[-1].pose.position.y - self.y
                self.expath_end_dis = math.hypot(ex, ey)

    def cmd_cb(self, msg):
        with self.lock:
            self.cmd_vx = msg.linear.x
            self.cmd_wz = msg.angular.z
            self.last_cmd_wall = rospy.get_time()

    def timer_cb(self, _event):
        with self.lock:
            stale = (self.last_cmd_wall is None or
                     rospy.get_time() - self.last_cmd_wall > 0.2)
            cmd_vx = 0.0 if stale else self.cmd_vx
            cmd_wz = 0.0 if stale else self.cmd_wz
            row = [
                rospy.get_rostime().to_sec(), self.t_ros,
                self.x, self.y, self.yaw,
                cmd_vx, cmd_wz,
                self.twist_vx, self.twist_wz,
                self.path_size, self.path_end_dis, self.path_heading_err,
                self.goal_dist, self.goal_heading_err,
                self.expath_size, self.expath_end_dis,
            ]
            self.msg_count += 1
        self.writer.writerow(row)

    def map_cb(self, msg):
        unknown = free = occ = 0
        for v in msg.data:
            if v == -1:
                unknown += 1
            elif v >= 50:
                occ += 1
            else:
                free += 1
        with self.lock:
            t = self.t_ros
        self.cov_writer.writerow(
            [t if t is not None else -1.0, unknown, free, occ])

    def flush_cb(self, _event):
        self.f.flush()
        if self.cov_f:
            self.cov_f.flush()

    def close(self):
        self.f.flush()
        self.f.close()
        if self.cov_f:
            self.cov_f.flush()
            self.cov_f.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", default="/out/diag.csv")
    ap.add_argument("--odom-topic", default="/ground_truth/state",
                    help="sim: /ground_truth/state; bag replay: /aft_mapped_to_init")
    ap.add_argument("--coverage-period", type=float, default=2.0,
                    help="seconds between /globalMap coverage samples; "
                         "0 disables coverage logging")
    args, _ = ap.parse_known_args()
    rospy.init_node("lrae_diag_logger")
    logger = DiagLogger(args.output, args.coverage_period, args.odom_topic)
    try:
        rospy.spin()
    finally:
        logger.close()
        rospy.loginfo("diag logger wrote %d rows", logger.msg_count)


if __name__ == "__main__":
    main()
