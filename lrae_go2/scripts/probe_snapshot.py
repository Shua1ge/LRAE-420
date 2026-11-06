#!/usr/bin/env python3
"""One-shot diagnostic snapshot: globalMap + plane_OccMap + cloud + pose.

Used to answer "points exist here but no map cell": the plane map says whether
fitplane produced a surface, the global map says whether merge kept it, and the
raw cloud says whether the points were even inside the z band.

Usage: probe_snapshot.py <out_prefix> [timeout_s]
Writes <prefix>_global.npz, <prefix>_plane.npz, <prefix>_cloud.npy, <prefix>_pose.npy
"""
import sys

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2


def main():
    prefix = sys.argv[1]
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

    box = {}
    order = ["global", "plane", "cloud", "pose"]

    rospy.init_node("probe_snapshot", anonymous=True)
    rospy.Subscriber("/globalMap", OccupancyGrid, lambda m: box.setdefault("global", m), queue_size=1)
    rospy.Subscriber("/plane_OccMap", OccupancyGrid, lambda m: box.setdefault("plane", m), queue_size=1)
    rospy.Subscriber("/cloud_registered", PointCloud2, lambda m: box.setdefault("cloud", m), queue_size=1)
    rospy.Subscriber("/aft_mapped_to_init", Odometry, lambda m: box.setdefault("pose", m), queue_size=1)

    while not rospy.is_shutdown() and len(box) < len(order):
        rospy.sleep(0.1)

    if "global" in box:
        m = box["global"]; i = m.info
        np.savez_compressed(prefix + "_global.npz",
                            data=np.asarray(m.data, dtype=np.int16),
                            width=i.width, height=i.height, resolution=i.resolution,
                            origin_x=i.origin.position.x, origin_y=i.origin.position.y)
    if "plane" in box:
        m = box["plane"]; i = m.info
        np.savez_compressed(prefix + "_plane.npz",
                            data=np.asarray(m.data, dtype=np.int16),
                            width=i.width, height=i.height, resolution=i.resolution,
                            origin_x=i.origin.position.x, origin_y=i.origin.position.y)
    if "cloud" in box:
        m = box["cloud"]
        f = {x.name: x for x in m.fields}
        dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                       "offsets": [f["x"].offset, f["y"].offset, f["z"].offset],
                       "itemsize": m.point_step})
        a = np.frombuffer(m.data, dtype=dt)
        p = np.stack((a["x"], a["y"], a["z"]), axis=1)
        p = p[np.isfinite(p).all(axis=1)]
        np.save(prefix + "_cloud.npy", p)
    if "pose" in box:
        q = box["pose"].pose.pose
        np.save(prefix + "_pose.npy", np.array([q.position.x, q.position.y, q.position.z]))
    print("[probe] saved %s: %s" % (prefix, ", ".join(k for k in order if k in box)))


if __name__ == "__main__":
    main()
