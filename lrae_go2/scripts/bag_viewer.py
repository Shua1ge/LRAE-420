#!/usr/bin/env python3
"""Helpers for viewing a converted Go2/MID-360 bag in RViz.

Publishes the recorded robot pose as an arrow marker and the trajectory as a
Path, so the bag can be inspected without running the LRAE stack.
"""
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker


class BagViewer:
    def __init__(self):
        self.trail = Path()
        self.trail.header.frame_id = "map"
        self.trail_pub = rospy.Publisher("/bag_trail", Path, queue_size=1)
        self.marker_pub = rospy.Publisher("/bag_pose_marker", Marker, queue_size=1)
        rospy.Subscriber("/aft_mapped_to_init", Odometry, self.cb, queue_size=20)
        rospy.loginfo("bag_viewer: publishing /bag_trail and /bag_pose_marker")

    def cb(self, msg):
        p = msg.pose.pose
        ps = PoseStamped()
        ps.header.stamp = msg.header.stamp
        ps.header.frame_id = "map"
        ps.pose = p
        self.trail.poses.append(ps)
        if len(self.trail.poses) > 20000:
            self.trail.poses.pop(0)
        self.trail.header.stamp = msg.header.stamp
        self.trail_pub.publish(self.trail)

        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = msg.header.stamp
        m.type = Marker.ARROW
        m.scale.x, m.scale.y, m.scale.z = 1.2, 0.18, 0.18
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.1, 1.0
        m.pose = p
        self.marker_pub.publish(m)


if __name__ == "__main__":
    rospy.init_node("bag_viewer")
    BagViewer()
    rospy.spin()
