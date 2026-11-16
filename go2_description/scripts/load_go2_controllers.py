#!/usr/bin/env python3
"""Load and start the two Go2 controllers after gazebo_ros_control appears.

The stock controller_manager/spawner races the Gazebo model spawn.  This node
waits for the controller_manager services and retries, which keeps the startup
script deterministic for headless experiments.
"""

import time

import rospy
from controller_manager_msgs.srv import (
    LoadController, LoadControllerRequest, SwitchController,
    SwitchControllerRequest)


def main():
    rospy.init_node("go2_controller_loader")
    controllers = rospy.get_param(
        "~controllers", ["joint_state_controller",
                         "joint_group_effort_controller"])
    load = rospy.ServiceProxy(
        "/controller_manager/load_controller", LoadController)
    switch = rospy.ServiceProxy(
        "/controller_manager/switch_controller", SwitchController)
    rospy.loginfo("waiting for controller_manager services")
    for _ in range(120):
        if rospy.is_shutdown():
            return
        try:
            load.wait_for_service(timeout=1.0)
            switch.wait_for_service(timeout=1.0)
            break
        except rospy.ROSException:
            continue
    else:
        rospy.logerr("controller_manager services did not appear")
        return
    for name in controllers:
        for _ in range(20):
            try:
                response = load(LoadControllerRequest(name=name))
                if response.ok:
                    rospy.loginfo("loaded controller %s", name)
                    break
            except rospy.ServiceException as exc:
                rospy.logwarn("load %s failed: %s", name, exc)
            time.sleep(0.5)
        else:
            rospy.logerr("could not load controller %s", name)
            return
    request = SwitchControllerRequest()
    request.start_controllers = list(controllers)
    request.stop_controllers = []
    request.strictness = SwitchControllerRequest.STRICT
    request.start_asap = True
    request.timeout = 5.0
    for _ in range(20):
        try:
            response = switch(request)
            if response.ok:
                rospy.loginfo("started controllers: %s", controllers)
                return
        except rospy.ServiceException as exc:
            rospy.logwarn("switch failed: %s", exc)
        time.sleep(0.5)
    rospy.logerr("could not start controllers")


if __name__ == "__main__":
    main()
