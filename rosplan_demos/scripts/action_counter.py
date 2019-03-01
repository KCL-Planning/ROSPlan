#!/usr/bin/env python
import rospy
import sys

from rosplan_dispatch_msgs.msg import ActionDispatch
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

count_ = 0

def callback(data):
    global count_
    count_ = count_ + 1
    
def handle_add_two_ints(req):
    global count_
    return TriggerResponse(True,str(count_))

def listener():
    count_ = 0
    rospy.init_node('action_counter', anonymous=True)
    s = rospy.Service('action_count', Trigger, handle_add_two_ints)
    rospy.Subscriber("/rosplan_plan_dispatcher/action_dispatch", ActionDispatch, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
