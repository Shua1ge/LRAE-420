#!/usr/bin/env python3
"""ROS 1 port of SCAN's MID360 point-cloud filtering for GBPlanner2.

This is a practical v2:
  * world-frame TF;
  * observed ground on a coarse XY grid (not a single body-height prior);
  * double-cylinder self filtering;
  * body-relative low/high separation at +/-0.25 m;
  * ground/obstacle bands 0.035/0.22/0.55 m;
  * high returns are retained only when a body-band obstacle exists in the same
    XY cell (wall), otherwise published to a debug high-surface topic;
  * D435 is voxel-downsampled before filtering;
  * max-range/no-return sentinels are discarded, never turned into obstacles.
"""

import math

import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def quat_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        raise ValueError("invalid quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def xyz_arrays(msg):
    fields = {field.name: field for field in msg.fields}
    if not all(name in fields for name in ("x", "y", "z")):
        raise ValueError("cloud lacks x/y/z")
    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    def read(name):
        offset = fields[name].offset
        return data[:, offset:offset + 4].copy().view("<f4").reshape(-1)
    return read("x"), read("y"), read("z")


def voxel_downsample(points, leaf):
    if len(points) == 0 or leaf <= 0.0:
        return points
    keys = np.floor(points / leaf).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[indices]


class Mid360Filter:
    def __init__(self):
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.topics = rospy.get_param(
            "~input_topics",
            ["/mid360_points", "/camera/depth/color/points"])
        self.max_range = float(rospy.get_param("~max_range", 40.0))
        self.no_return_range = float(rospy.get_param("~no_return_range", 3.0))
        self.nominal_body_height = float(
            rospy.get_param("~nominal_body_height", 0.33))
        self.self_radius = float(rospy.get_param("~self_radius", 0.25))
        self.self_offset = float(rospy.get_param("~self_offset", 0.18))
        self.self_z_down = float(rospy.get_param("~self_z_down", 0.45))
        self.self_z_up = float(rospy.get_param("~self_z_up", 0.20))
        self.low_below_body = float(rospy.get_param("~low_below_body", 0.25))
        self.high_above_body = float(rospy.get_param("~high_above_body", 0.25))
        self.ground_band = float(rospy.get_param("~ground_band", 0.035))
        self.obstacle_min = float(rospy.get_param("~obstacle_min", 0.22))
        self.obstacle_max = float(rospy.get_param("~obstacle_max", 0.55))
        self.ground_cell = float(rospy.get_param("~ground_cell", 0.40))
        self.ground_prior = float(rospy.get_param("~ground_prior", 0.33))
        self.d435_voxel = float(rospy.get_param("~d435_voxel", 0.06))
        self.support_cell = float(rospy.get_param("~support_cell", 0.30))
        self.body = None
        self.ground = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            "/input_pointcloud", PointCloud2, queue_size=2)
        self.high_publisher = rospy.Publisher(
            "/mid360_filter/high_surface", PointCloud2, queue_size=2)
        rospy.Subscriber(
            "/ground_truth/state", Odometry, self.body_callback, queue_size=10)
        for topic in self.topics:
            rospy.Subscriber(
                topic, PointCloud2,
                lambda msg, t=topic: self.cloud_callback(msg, t),
                queue_size=2)
        rospy.loginfo(
            "MID360 filter v2: %s -> /input_pointcloud, self=(r=%.2f,o=%.2f,"
            "z=%.2f/%.2f), bands=%.3f/%.2f/%.2f, ground_cell=%.2f, "
            "d435_voxel=%.2f", self.topics, self.self_radius,
            self.self_offset, self.self_z_down, self.self_z_up,
            self.ground_band, self.obstacle_min, self.obstacle_max,
            self.ground_cell, self.d435_voxel)

    def body_callback(self, msg):
        p = msg.pose.pose
        q = p.orientation
        self.body = (p.position.x, p.position.y, p.position.z,
                     math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def transform(self, msg):
        for stamp in (msg.header.stamp, rospy.Time(0)):
            try:
                return self.tf_buffer.lookup_transform(
                    self.world_frame, msg.header.frame_id, stamp,
                    rospy.Duration(0.10))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue
        return None

    @staticmethod
    def key(x, y, cell):
        return (int(math.floor(x / cell)), int(math.floor(y / cell)))

    @staticmethod
    def encode(keys):
        return keys[:, 0].astype(np.int64) * 1000003 + keys[:, 1].astype(np.int64)

    def update_ground(self, points, body_z):
        # Low measured points are the ground candidates.  A 20th percentile per
        # coarse cell is robust to a few leg/rock returns; the estimate is
        # smoothed over time and clipped near the body-height prior.
        mask = points[:, 2] < body_z - self.low_below_body
        if not np.any(mask):
            return
        low = points[mask]
        keys = np.floor(low[:, :2] / self.ground_cell).astype(np.int64)
        unique, inverse = np.unique(keys, axis=0, return_inverse=True)
        order = np.argsort(inverse)
        bounds = np.searchsorted(inverse[order], np.arange(len(unique)))
        ends = np.r_[bounds[1:], len(order)]
        prior = body_z - self.ground_prior
        for cell, start, end in zip(unique, bounds, ends):
            values = low[order[start:end], 2]
            estimate = float(np.percentile(values, 20.0))
            estimate = float(np.clip(estimate, prior - 0.60, prior + 0.60))
            key = (int(cell[0]), int(cell[1]))
            if key in self.ground:
                self.ground[key] = 0.80 * self.ground[key] + 0.20 * estimate
            else:
                self.ground[key] = estimate

    def ground_at(self, points, body_z):
        keys = np.floor(points[:, :2] / self.ground_cell).astype(np.int64)
        unique, inverse = np.unique(keys, axis=0, return_inverse=True)
        values = np.array([
            self.ground.get((int(cell[0]), int(cell[1])),
                            body_z - self.ground_prior)
            for cell in unique
        ], dtype=np.float64)
        return values[inverse]

    def publish(self, publisher, points, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = self.world_frame
        publisher.publish(point_cloud2.create_cloud_xyz32(
            header, points.astype(np.float32).tolist()))

    def cloud_callback(self, msg, topic):
        body = self.body
        if body is None:
            return
        try:
            x, y, z = xyz_arrays(msg)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "cloud decode failed: %s", exc)
            return
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        raw_range = np.sqrt(x * x + y * y + z * z)
        # Gazebo Classic PointCloud2 has no explicit invalid/no-return flag.
        # Drop max-range sentinels; never create a synthetic 3 m obstacle.
        valid &= raw_range < self.max_range - 0.10
        points = np.column_stack((x[valid], y[valid], z[valid]))
        if len(points) == 0:
            return
        tf = self.transform(msg)
        if tf is None:
            rospy.logwarn_throttle(2.0, "no TF; dropping scan")
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        rotation = quat_matrix(q.x, q.y, q.z, q.w)
        world = points @ rotation.T + np.array([t.x, t.y, t.z])
        raw_count = len(world)
        if "camera" in topic or len(world) > 50000:
            world = voxel_downsample(world, self.d435_voxel)
        bx, by, bz, yaw = body
        self.update_ground(world, bz)

        heading = np.array([math.cos(yaw), math.sin(yaw)])
        rel_xy = world[:, :2] - np.array([bx, by])
        front = rel_xy - self.self_offset * heading
        rear = rel_xy + self.self_offset * heading
        inside_xy = ((front[:, 0] ** 2 + front[:, 1] ** 2) <= self.self_radius ** 2) | \
                    ((rear[:, 0] ** 2 + rear[:, 1] ** 2) <= self.self_radius ** 2)
        rel_z = world[:, 2] - bz
        inside = inside_xy & (rel_z >= -self.self_z_down) & (rel_z <= self.self_z_up)

        ground_z = self.ground_at(world, bz)
        height = world[:, 2] - ground_z
        ground = (height >= -self.ground_band) & (height <= self.ground_band)
        low = rel_z < -self.low_below_body
        high = rel_z > self.high_above_body
        obstacle = (~inside) & (~high) & \
            (height >= self.obstacle_min) & (height <= self.obstacle_max)

        # A high return is kept only if a body-band obstacle exists in the same
        # or adjacent XY support cell.  That preserves the upper part of a wall
        # while excluding a ceiling/arch over empty floor.
        body_mask = obstacle & ~inside
        support_keys = np.floor(
            world[body_mask, :2] / self.support_cell).astype(np.int64)
        support_encoded = set(self.encode(support_keys).tolist())
        high_indices = np.flatnonzero(high)
        high_keys = np.floor(
            world[high_indices, :2] / self.support_cell).astype(np.int64)
        keep_high = np.zeros(len(world), dtype=bool)
        if len(high_keys):
            high_encoded = self.encode(high_keys)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    candidate = high_encoded + dx * 1000003 + dy
                    keep_high[high_indices] |= np.array(
                        [int(v) in support_encoded for v in candidate],
                        dtype=bool)

        keep = ground | obstacle | keep_high
        keep &= ~(inside & ~ground)
        keep &= ~(low & ~ground)
        keep &= ~(high & ~keep_high)
        result = world[keep]
        self.publish(self.publisher, result, msg.header.stamp)
        self.publish(self.high_publisher, world[high], msg.header.stamp)
        rospy.loginfo_throttle(
            5.0,
            "filter %s: raw=%d used=%d out=%d ground=%d obstacle=%d "
            "high_wall=%d high_drop=%d self=%d",
            topic, raw_count, len(world), len(result), int(ground.sum()),
            int(obstacle.sum()), int(keep_high.sum()),
            int((high & ~keep_high).sum()), int(inside.sum()))


if __name__ == "__main__":
    rospy.init_node("mid360_filter")
    Mid360Filter()
    rospy.spin()
