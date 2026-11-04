#!/usr/bin/env python3
"""Measure publish rates and message sizes for the LRAE + Go2 stack.

Subscribes to the key topics for --duration seconds, then prints one CSV row
per topic: rate (Hz), interval p50/p90 (ms), message size (points/bytes), and
count.  Designed for a steady-state window (start after LRAE is exploring).
"""

import argparse
import threading
import time

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import PointCloud2

TOPICS = [
    ("/mid360_points", PointCloud2, "cloud"),
    ("/mid360_world", PointCloud2, "cloud"),
    ("/plane_OccMap", OccupancyGrid, "grid"),
    ("/plane_map", PointCloud2, "cloud"),
    ("/local_traversibility_ponit_cloud", PointCloud2, "cloud"),
    ("/globalMap", OccupancyGrid, "grid"),
    ("/path", Path, "path"),
    ("/look_ahead_goal", PointStamped, "point"),
    ("/exporation_path", Path, "path"),
    ("/planning/cmd_vel_raw", Twist, "twist"),
    ("/ground_truth/state", Odometry, "odom"),
    ("/joint_states", None, "joint"),
]


class Recorder:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {t[0]: {"times": [], "sz": None} for t in TOPICS}
        self.start = time.time()
        self.stop = False

    def make_cb(self, topic, kind):
        def cb(msg):
            now = time.time()
            with self.lock:
                d = self.data[topic]
                d["times"].append(now)
                if kind == "cloud":
                    n = msg.width * msg.height
                    d["sz"] = (n, len(msg.data))
                elif kind == "grid":
                    d["sz"] = (msg.info.width, msg.info.height, msg.info.resolution)
                elif kind == "path":
                    d["sz"] = len(msg.poses)
                elif kind == "joint":
                    d["sz"] = len(msg.position)
        return cb

    def report(self, duration):
        import numpy as np
        print(f"\n=== measured over {duration:.0f} s ===")
        print(f"{'topic':38s} {'rate(Hz)':>9s} {'p50(ms)':>8s} {'p90(ms)':>8s}  detail")
        for topic, _, kind in TOPICS:
            d = self.data[topic]
            ts = d["times"]
            if len(ts) < 5:
                print(f"{topic:38s} {'-':>9s} {'-':>8s} {'-':>8s}  (no data)")
                continue
            # discard the first sample (startup transient)
            ts = ts[1:]
            dt = np.diff(ts) * 1000.0
            rate = (len(ts) - 1) / (ts[-1] - ts[0])
            detail = ""
            if kind == "cloud" and d["sz"]:
                n, b = d["sz"]
                detail = f"{n} pts, {b/1e6:.2f} MB"
            elif kind == "grid" and d["sz"]:
                w, h, res = d["sz"]
                detail = f"{w}x{h} @ {res:.2f} m"
            elif kind == "path" and d["sz"] is not None:
                detail = f"{d['sz']} poses"
            elif kind == "twist":
                detail = "cmd"
            elif kind == "odom":
                detail = "gt odom"
            elif kind == "joint":
                detail = f"{d['sz']} joints"
            print(f"{topic:38s} {rate:9.1f} {np.median(dt):8.1f} "
                  f"{np.percentile(dt, 90):8.1f}  {detail}")
        print(f"{'/joint_states':38s} (rate above is callback rate)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start-delay", type=float, default=45.0,
                    help="wait before subscribing (let LRAE start moving)")
    ap.add_argument("--duration", type=float, default=30.0)
    args, _ = ap.parse_known_args()
    rospy.init_node("rate_probe")
    time.sleep(args.start_delay)

    rec = Recorder()
    from sensor_msgs.msg import JointState
    subs = []
    for topic, msg_type, kind in TOPICS:
        if msg_type is None:
            msg_type = JointState
        subs.append(rospy.Subscriber(topic, msg_type,
                                     rec.make_cb(topic, kind), queue_size=200))
    rospy.loginfo("rate probe recording for %.0f s", args.duration)
    time.sleep(args.duration)
    for s in subs:
        s.unregister()
    rec.report(args.duration)


if __name__ == "__main__":
    main()
