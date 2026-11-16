#!/usr/bin/env python3
"""Broadcast world -> base from the Gazebo ground-truth odometry."""

import rospy
import tf
from nav_msgs.msg import Odometry


class GroundTruthTf:
    def __init__(self):
        self.broadcaster = tf.TransformBroadcaster()
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.world_frame = rospy.get_param("~world_frame", "world")
        rospy.Subscriber(
            "/ground_truth/state", Odometry, self.odom_callback, queue_size=10)

    def odom_callback(self, msg):
        pose = msg.pose.pose
        stamp = msg.header.stamp
        if stamp.is_zero():
            stamp = rospy.Time.now()
        self.broadcaster.sendTransform(
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w),
            stamp, self.base_frame, self.world_frame)


if __name__ == "__main__":
    rospy.init_node("go2_ground_truth_tf")
    GroundTruthTf()
    rospy.spin()
