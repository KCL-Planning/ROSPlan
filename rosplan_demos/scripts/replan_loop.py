#!/usr/bin/env python
import rospkg
import rospy
import sys
import random

from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse
from rosplan_dispatch_msgs.srv import ExecAlternatives, ExecAlternativesResponse

from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayResponse, KnowledgeUpdateServiceArrayRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue

import os

def perturb():
    try:
        update = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
        arr = KnowledgeUpdateServiceArrayRequest()
        update_type = []
        call = False

        for i in [0,1,2]:

            if random.randint(1,100) <= 50:

                ki_remove = KnowledgeItem()
                ki_remove.knowledge_type = 1
                ki_remove.attribute_name = 'machine_off'
                kv = KeyValue()
                kv.key = 'm'
                kv.value =  'machine'+str(i)
                ki_remove.values.append(kv)
                arr.knowledge.append(ki_remove)
                update_type.append(2)

                ki_remove = KnowledgeItem()
                ki_remove.knowledge_type = 1
                ki_remove.attribute_name = 'machine_on'
                kv = KeyValue()
                kv.key = 'm'
                kv.value =  'machine'+str(i)
                ki_remove.values.append(kv)
                arr.knowledge.append(ki_remove)
                update_type.append(0)

                call = True

        if call:
            arr.update_type = update_type
            update(arr)

    except rospy.ServiceException, e:
        print 'Unexpected perturbances not called'

def run():
    rospy.init_node('coordinator', anonymous=False)

    # use or not adaptable plan dispatcher
    adaptable_plan_dispatcher_required = rospy.get_param('~adaptable_plan_dispatcher_required', True)
    if adaptable_plan_dispatcher_required:
        print 'using adaptable plan dispatcher'
    else:
        print 'NOT using adaptable plan dispatcher'

    # for logging purposes, write results of the experiment to a file
    ros_tcp_port = os.environ['ROS_MASTER_URI'].replace('http://localhost:', '')
    if adaptable_plan_dispatcher_required:
        log_file = open('exp_results_adaptable_' + ros_tcp_port + '.csv','w')
    else:
        log_file = open('exp_results_non_adaptable_' + ros_tcp_port + '.csv','w')
    log_file.write('succeeded?, number of replans, number of executed actions\n')

    goal_achieved = False
    replans = 0

    while not goal_achieved and replans<25:
        rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
        rospy.wait_for_service('/rosplan_planner_interface/planning_server')
        rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
        rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
        rospy.wait_for_service('/rosplan_knowledge_base/update_array')
        try:
            pg = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
            pg()

            pi = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
            pi()

            pp = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan', Empty)
            pp()

            if(adaptable_plan_dispatcher_required):
                ea = rospy.ServiceProxy('/csp_exec_generator/gen_exec_alternatives', ExecAlternatives)
                ear = ea()

            perturb()

            dp = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
            dsr = dp()

            goal_achieved = dsr.goal_achieved
            if not dsr.goal_achieved:
                replans += 1

        except rospy.ServiceException, e:
            replans += 1

    # get number of executed actions
    number_of_executed_actions = 0
    try:
        ac = rospy.ServiceProxy('/action_count', Trigger)
        acr = ac()
        print 'Actions: ' + acr.message
        number_of_executed_actions = acr.message
    except rospy.ServiceException, e:
        print 0

    # check if goal was achieved, write to log file
    if goal_achieved:
        print 'SUCCESS ', str(replans)
        log_file.write('true, ' + str(replans) + ', ' + str(number_of_executed_actions) + '\n')
    else:
        print 'FAILED ', str(replans)
        log_file.write('false' + str(replans) + ', ' + str(number_of_executed_actions) + '\n')

    # for logging purposes, write experiment results to text file, closing the file since we are done
    log_file.close()


if __name__ == '__main__':
    run()
