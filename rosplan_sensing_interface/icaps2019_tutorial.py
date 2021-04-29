#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from math import sqrt

def robot_at(msg, params): 

    assert(msg.header.frame_id == "map")
    assert(len(params) == 2)
    ret_value = []
    attributes = get_kb_attribute("robot_at")
    curr_wp = ''

    # Find current robot_location in knowledge base
    for a in attributes:
        if not a.is_negative:
            curr_wp = a.values[1].value
            break

    # for each robot parameter
    for robot in params[0]:

        # find closest waypoint
        distance = float('inf')
        closest_wp = ''
        for wp in params[1]:
            pose = rospy.get_param("/rosplan_demo_waypoints/"+wp)
            assert(len(pose) > 0)
            x = pose[0] - msg.pose.pose.position.x
            y = pose[1] - msg.pose.pose.position.y
            d = sqrt(x**2 + y**2)
            if d < distance:
                closest_wp = wp
                distance = d

        # set state in knowledge base
        if curr_wp != closest_wp:
            ret_value.append((robot + ':' + curr_wp, False)) # Set current waypoint to false
            ret_value.append((robot + ':' + closest_wp, True))  # Set new wp to true

    return ret_value
