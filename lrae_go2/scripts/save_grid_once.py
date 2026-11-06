#!/usr/bin/env python3
"""Save one sample of an OccupancyGrid topic to .npz.

Used for offline diagnosis: the global map is 1 MB per message, so recording it
at 10 Hz for a whole run wastes gigabytes.  One snapshot plus the local
plane_OccMap time series is enough to tell a stale cell from a live obstacle.

readiness: wait_for_message blocks on wall time, so it works even though the
stack runs with use_sim_time=true (a rospy.Time deadline would expire the
moment the bag clock passes it).

Usage: save_grid_once.py /globalMap /out/global_map.npz [timeout_s]
"""
import sys

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid


def main():
    topic = sys.argv[1]
    path = sys.argv[2]
    timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0

    rospy.init_node("save_grid_once", anonymous=True)
    msg = rospy.wait_for_message(topic, OccupancyGrid, timeout=timeout)
    info = msg.info
    np.savez_compressed(
        path,
        data=np.asarray(msg.data, dtype=np.int16),
        width=info.width, height=info.height,
        resolution=info.resolution,
        origin_x=info.origin.position.x, origin_y=info.origin.position.y,
        stamp=msg.header.stamp.to_sec(), frame=msg.header.frame_id)
    print("[save_grid_once] %s %dx%d res=%.2f origin=(%.1f,%.1f) -> %s" %
          (topic, info.width, info.height, info.resolution,
           info.origin.position.x, info.origin.position.y, path))


if __name__ == "__main__":
    main()
