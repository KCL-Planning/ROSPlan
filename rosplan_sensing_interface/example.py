#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from math import sqrt


def robot_at_test(msg, params): # Idea: Params are the instances of all the parameters in a dictionary and the messaestoreproxy
    assert(msg.header.frame_id == "map")
    assert(len(params) == 2)
    ret_value = []
    for robot in params[0]:
        distance = float('inf')
        closest_wp = 0
        for i, wp in enumerate(params[1]):
            ret_value.append((robot + ':' + wp, False))  # Init all the values
            pose = rospy.get_param("/rosplan_demo_waypoints/" + wp)
            assert(len(pose) > 0)
            x = pose[0].pose.position.x - msg.pose.pose.position.x
            y = pose[0].pose.position.y - msg.pose.pose.position.y
            d = sqrt(x**2 + y**2)
            if d < distance:
                closest_wp = i
                distance = d
        ret_value[closest_wp] = (ret_value[closest_wp][0], True)
    return ret_value


def robot_at(msg, params): # Idea: Params are the instances of all the parameters in a dictionary and the messaestoreproxy
    assert(msg.header.frame_id == "map")
    assert(len(params) == 2)
    ret_value = []
    attributes = get_kb_attribute("robot_at")
    curr_wp = ''
    # Find current robot_location
    for a in attributes:
        if not a.is_negative:
            curr_wp = a.values[1].value
            break

    for robot in params[0]:
        distance = float('inf')
        closest_wp = ''
        for wp in params[1]:
            pose = rospy.get_param("/rosplan_demo_waypoints/"+wp)
            assert(len(pose) > 0)
            x = pose[0].pose.position.x - msg.pose.pose.position.x
            y = pose[0].pose.position.y - msg.pose.pose.position.y
            d = sqrt(x**2 + y**2)
            if d < distance:
                closest_wp = wp
                distance = d
        if curr_wp != closest_wp:
            ret_value.append((robot + ':' + curr_wp, False)) # Set current waypoint to false
            ret_value.append((robot + ':' + closest_wp, True))  # Set new wp to true
    return ret_value


#from std_srvs.srv import SetBoolRequest
def req_docked():
    return SetBoolRequest(data=False)

def docked(res, params):  # params is a list with all the parameters - fully instantiated for services!
    print(params)
    return int(res.message.split(' ')[3])%2 == 0
