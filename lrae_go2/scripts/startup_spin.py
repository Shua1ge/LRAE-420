#!/usr/bin/env python3
"""Slow 360 deg spin-in-place before handing control to pathFollower.

Why: the MID360 cannot see the ground inside ~2.1 m (blind cone) and only
samples the ground in a few annuli; a slow spin at startup observes the full
2.1-11 m ring in every direction before the planner commits to a frontier.

Publishes to /planning/cmd_vel_raw.  Launch it so it runs BEFORE pathFollower
(the follower's start delay must be offset by the spin duration).
"""

import math

import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node("startup_spin")
    topic = rospy.get_param("~topic", "/planning/cmd_vel_raw")
    rate_hz = float(rospy.get_param("~rate", 0.6))          # rad/s
    duration = float(rospy.get_param("~duration", 10.5))    # 2*pi/0.6 ~ 10.5 s
    settle = float(rospy.get_param("~settle", 1.0))
    pub = rospy.Publisher(topic, Twist, queue_size=1)
    rospy.loginfo("startup_spin: %.1f rad/s for %.1f s on %s",
                  rate_hz, duration, topic)
    r = rospy.Rate(20.0)
    cmd = Twist()
    cmd.angular.z = rate_hz
    t_end = rospy.get_rostime().to_sec() + duration
    while not rospy.is_shutdown() and rospy.get_rostime().to_sec() < t_end:
        pub.publish(cmd)
        r.sleep()
    stop = Twist()
    t_end = rospy.get_rostime().to_sec() + settle
    while not rospy.is_shutdown() and rospy.get_rostime().to_sec() < t_end:
        pub.publish(stop)
        r.sleep()
    rospy.loginfo("startup_spin: done")


if __name__ == "__main__":
    main()
