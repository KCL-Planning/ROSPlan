#!/usr/bin/env python3

import rospy
import genpy
import yaml

from std_msgs.msg import String
from rosplan_dispatch_msgs.msg import EsterelPlan


"""
Read yaml-string representing a esterel plan from the planner output topic and publish it again as an esterel message.
"""
class DirectEsterelParser:

    def __init__(self):
        plan_topic = "~" + rospy.get_param("~plan_topic", "complete_plan")
        self.pub_complete_plan = rospy.Publisher(plan_topic, EsterelPlan, queue_size=5)

        planner_output_topic = rospy.get_param("~planner_topic", "planner_output")
        rospy.Subscriber(planner_output_topic, String, self.planner_output_cb, queue_size=10)

    def planner_output_cb(self, msg):
        rospy.loginfo("DirectEsterelParser: Received new plan")
        esterel = EsterelPlan()
        genpy.message.fill_message_args(esterel, yaml.load(msg.data))
        self.pub_complete_plan.publish(esterel)


def main():
    rospy.init_node('rosplan_parsing_interface')
    parser = DirectEsterelParser()
    rospy.spin()


if __name__ == "__main__":
    main()
