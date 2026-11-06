#!/usr/bin/env python3
"""Union every /cloud_registered frame of a ROS1 bag into one PCD, offline.

/cloud_registered from FAST-LIVO2 is already registered in the SLAM map frame,
so the union of the frames *is* the map the recording produced: no fitplane, no
exploration_map_merge, no planner.  Runs directly on the bag file (no roscore,
no /clock, no TF), so it is much faster than replaying in real time.

Usage:
  python3 bag_map_dump.py --bag /data/x.bag --out /out/bag_map.pcd
  python3 bag_map_dump.py ... --voxel 0.10 --max-frames 200   # quick check
"""

import argparse
import os
import sys
import time

import numpy as np
import rosbag

FLOAT32 = 7


def xyz_from_msg(msg):
    """Vectorised xyz extraction; returns Nx3 float32 or None."""
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return None
    if any(fields[k].datatype != FLOAT32 for k in ("x", "y", "z")):
        return None
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": ["<f4", "<f4", "<f4"],
        "offsets": [fields["x"].offset, fields["y"].offset, fields["z"].offset],
        "itemsize": msg.point_step,
    })
    cloud = np.frombuffer(msg.data, dtype=dtype)
    xyz = np.stack((cloud["x"], cloud["y"], cloud["z"]), axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]


def unique_rows(keys):
    """np.unique(axis=0) is very slow on millions of rows; compare as bytes."""
    keys = np.ascontiguousarray(keys)
    view = keys.view(np.dtype((np.void, keys.dtype.itemsize * keys.shape[1])))
    _, idx = np.unique(view.ravel(), return_index=True)
    return keys[idx]


def write_pcd(path, points):
    n = points.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        "WIDTH %d\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %d\n"
        "DATA binary\n" % (n, n)
    )
    with open(path, "wb") as handle:
        handle.write(header.encode("ascii"))
        points.astype("<f4").tofile(handle)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--topic", default="/cloud_registered")
    parser.add_argument("--voxel", type=float, default=0.15,
                        help="voxel size in metres for the union")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="stop after N cloud frames (0 = whole bag)")
    parser.add_argument("--compact-every", type=int, default=200,
                        help="merge pending frames into the running set every N")
    args = parser.parse_args()

    if not os.path.isfile(args.bag):
        sys.exit("bag not found: %s" % args.bag)

    bag = rosbag.Bag(args.bag, "r")
    info = bag.get_type_and_topic_info().topics.get(args.topic)
    if info is None:
        sys.exit("topic %s not in bag (topics: %s)" %
                 (args.topic, ", ".join(sorted(bag.get_type_and_topic_info().topics))))
    print("[bag_map] %s: %d messages on %s, voxel %.2f m" %
          (os.path.basename(args.bag), info.message_count, args.topic, args.voxel))
    sys.stdout.flush()

    running = None
    pending = []
    frames = 0
    raw = 0
    t0 = time.time()

    def compact():
        nonlocal running, pending
        if not pending:
            return
        merged = pending[0] if len(pending) == 1 else np.concatenate(pending)
        if running is not None:
            merged = np.concatenate((running, merged))
        running = unique_rows(merged)
        pending = []

    try:
        for _, msg, _ in bag.read_messages(topics=[args.topic]):
            xyz = xyz_from_msg(msg)
            frames += 1
            if xyz is not None and xyz.size:
                raw += xyz.shape[0]
                keys = np.floor(xyz / args.voxel).astype(np.int32)
                pending.append(unique_rows(keys))
            if frames % args.compact_every == 0:
                compact()
                print("[bag_map] %d frames, %.1fM raw points, %.2fM voxels, %.0f s" %
                      (frames, raw / 1e6,
                       (0 if running is None else running.shape[0]) / 1e6,
                       time.time() - t0))
                sys.stdout.flush()
            if args.max_frames and frames >= args.max_frames:
                break
    finally:
        bag.close()
    compact()

    if running is None or running.shape[0] == 0:
        sys.exit("no points accumulated")

    centers = (running.astype(np.float64) + 0.5) * args.voxel
    centers = centers.astype(np.float32)
    write_pcd(args.out, centers)

    lo = centers.min(axis=0)
    hi = centers.max(axis=0)
    stats = (
        "frames=%d raw_points=%d voxels=%d voxel=%.3f m\n"
        "bbox_min=%.2f %.2f %.2f\nbbox_max=%.2f %.2f %.2f\n"
        "size_m=%.1f %.1f %.1f\nelapsed=%.0f s\npcd=%s\n" %
        (frames, raw, centers.shape[0], args.voxel,
         lo[0], lo[1], lo[2], hi[0], hi[1], hi[2],
         hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2],
         time.time() - t0, args.out)
    )
    with open(args.out + ".stats.txt", "w") as handle:
        handle.write(stats)
    print("[bag_map] done\n" + stats)


if __name__ == "__main__":
    main()
