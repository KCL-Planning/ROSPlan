#!/usr/bin/env python
import rospkg
import rospy
import sys

from std_srvs.srv import Empty, EmptyResponse
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse
from rosplan_dispatch_msgs.srv import ExecAlternatives, ExecAlternativesResponse

goal_achieved = False
replans = 0
while not goal_achieved and replans<25:
    rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
    rospy.wait_for_service('/rosplan_planner_interface/planning_server')
    rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
    rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
    try:
        pg = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
        pg()

        pi = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
        pi()

        pp = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan', Empty)
        pp()

        ea = rospy.ServiceProxy('/csp_exec_generator/gen_exec_alternatives', ExecAlternatives)
        ear = ea()

        dp = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
        dsr = dp()

        goal_achieved = dsr.goal_achieved
        if not dsr.goal_achieved:
            replans += 1

    except rospy.ServiceException, e:
        replans += 1

if goal_achieved:
    print "SUCCESS ", str(replans)
else:
    print "FAILED ", str(replans)
