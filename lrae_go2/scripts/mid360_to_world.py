#!/usr/bin/env python3
"""Transform the simulated MID-360 cloud into the world/map frame.

The Gazebo MID-360 model publishes /mid360_points in ``lidar_link``.  LRAE's
fitplane node does not transform its input and assumes the points are already in
``map``.  This node is the only adapter needed between the Go2 simulation and
LRAE: it looks up ``target_frame`` -> cloud frame at the cloud timestamp and
republishes XYZ points in the target frame.

It deliberately does no filtering: fitplane needs the ground points to estimate
the local support plane.  The GBPlanner-oriented mid360_filter.py removes them
and must not be used here.
"""

import numpy as np
import rospy
import tf
import tf2_ros
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField


class Mid360ToWorld:
    def __init__(self):
        self.source_topic = rospy.get_param("~source_topic", "/mid360_points")
        self.output_topic = rospy.get_param("~output_topic", "/mid360_world")
        self.target_frame = rospy.get_param("~target_frame", "world")
        self.tf_timeout = rospy.Duration(
            rospy.get_param("~tf_timeout", 0.10))

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
        ]
        self.pub = rospy.Publisher(
            self.output_topic, PointCloud2, queue_size=2)
        self.sub = rospy.Subscriber(
            self.source_topic, PointCloud2, self.cloud_callback, queue_size=2)
        rospy.loginfo(
            "mid360_to_world: %s (%s) -> %s (%s)",
            self.source_topic, "cloud frame", self.output_topic,
            self.target_frame)

    def cloud_callback(self, msg):
        source_frame = msg.header.frame_id
        if not source_frame:
            rospy.logwarn_throttle(5.0, "mid360_to_world: cloud has no frame_id")
            return

        stamp = msg.header.stamp
        if stamp.is_zero():
            stamp = rospy.Time(0)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, stamp, self.tf_timeout)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(
                2.0, "mid360_to_world: TF %s -> %s failed: %s",
                self.target_frame, source_frame, exc)
            return

        try:
            points = np.asarray(
                list(point_cloud2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=True)),
                dtype=np.float64)
        except Exception as exc:  # malformed cloud from the simulator
            rospy.logwarn_throttle(5.0, "mid360_to_world: read_points failed: %s", exc)
            return

        if points.size == 0:
            return
        points = points.reshape(-1, 3)

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = tf.transformations.quaternion_matrix(
            [rotation.x, rotation.y, rotation.z, rotation.w])
        rot = matrix[:3, :3]
        trans = np.array(
            [translation.x, translation.y, translation.z], dtype=np.float64)
        points = points @ rot.T + trans

        header = msg.header
        header.frame_id = self.target_frame
        out = point_cloud2.create_cloud(
            header, self.fields, points.astype(np.float32))
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("mid360_to_world")
    Mid360ToWorld()
    rospy.spin()
