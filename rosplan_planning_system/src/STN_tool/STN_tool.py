#!/usr/bin/env python3

import rospy
import re
import rospkg
import sys

from rosplan_dispatch_msgs.msg import *
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
from rosplan_planning_system.pyrobustenvelope import compute_envelope_construct, compute_envelope
import time


class RobustEnvelope(object):

    def __init__(self):

        # get path of pkg
        rospack = rospkg.RosPack()

        # get the parameters for the STN tool from the launch file 
        self.domain_path = rospy.get_param('~robust_domain_path', "domain.pddl")#.split('/')[-1]
        self.problem_path = rospy.get_param('~problem_path', "problem.pddl")#.split('/')[-1]
        self.STN_plan_path = self.problem_path [:-5] + "_plan.stn" #self.data_path + 'STN_plan_' + problem_name[:-5] + '.stn'
        self.Esterel_plan_path = self.problem_path [:-5] + "_plan.strl" #self.data_path + 'Esterel_plan_' + problem_name[:-5] +'.txt'
        self.max_parameters = rospy.get_param('~max_parameters', 1)
        self.stn_timeout = rospy.get_param('~stn_timeout', 60)

        #relate the parameter to the source node and the sink node
        self.dict_pram_source_node = dict()
        self.dict_pram_sink_node = dict()
        self.dict_params = dict()

        #getting the value of the dur from service
        self.dict_dur_lower = dict()
        self.dict_dur_upper = dict()

        #mapping the stn node to esterel node
        self.dict_stn = dict()

        # publications
        self.pub_robust_plan = rospy.Publisher(rospy.get_param('~robust_plan_topic'), EsterelPlan , queue_size=10)

        # subscriptions
        rospy.Subscriber(rospy.get_param('~plan_topic'), EsterelPlan, self.esterelPlanCallback, queue_size=1)

        # STN service offered by this node
        rospy.Service('run_STN', Empty, self.serviceCB)

        # Var for better readability
        self.esterel_plan_received = False
        self.output_robust_plan_msg = None
        self.publish_robust = False

        # to control the frequency at which this node will run
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # give some time for the node to subscribe to the topic
        rospy.sleep(0.2)
        rospy.loginfo('KCL: (' + rospy.get_name() + ') Using domain: ' + self.domain_path)
        rospy.loginfo('KCL: (' + rospy.get_name() + ') Using problem: ' + self.problem_path)
        rospy.loginfo('KCL: (' + rospy.get_name() + ') Ready to compute robust envelopes')

        self.best_rect = None


    def esterelPlanCallback(self, input_esterel_plan):
        # save in member variable
        if input_esterel_plan:
            self.output_robust_plan_msg = input_esterel_plan
        # raise flag indicating that msg has been received
            self.esterel_plan_received = True

            stn_plan_file = open(self.STN_plan_path, 'w')

            # print the actions
            for node in input_esterel_plan.nodes:
                if node.node_type == 0: # ACTION_START
                    pddl_action = "(" + node.action.name
                    for param in node.action.parameters:
                        pddl_action = pddl_action + " " + param.value
                    pddl_action = pddl_action + ")"
                    stn_plan_file.write("durative-action instance n" + str(node.node_id) + " : " + pddl_action + ";\n")

            # print the constraints
            parameter_count = 0
            for edge in input_esterel_plan.edges:
                
                # parameterized the action that we want in the Esterel plan (which corresponds with the parameterized action in the robust_domain 
                if parameter_count < self.max_parameters and edge.edge_type == 1 and input_esterel_plan.nodes[edge.source_ids[0]].action.name.startswith("collect"): # in here the actions that starts with "collect" will be parametrized/ can be replaced with any other action
                    stn_plan_file.write("parameter dur_" + str(parameter_count) + " default " + str(edge.duration_lower_bound) + ";\n")
                    self.dict_params["dur_" + str(parameter_count)] = edge.edge_id
                    bounds = "[dur_" + str(parameter_count) + ", dur_" + str(parameter_count) + "]"
                    parameter_count += 1
                else:
                    bounds = "[" + str(edge.duration_lower_bound) + ", "
                    if edge.duration_upper_bound >= sys.float_info.max:
                        bounds = bounds + "inf]"
                    else:
                        bounds = bounds + str(edge.duration_upper_bound) + "]"
                
                # prepare the stn plan based on Esterel plan 
                source = ".zero"
                if input_esterel_plan.nodes[edge.source_ids[0]].node_type == 0: # ACTION_START
                    source = "n" + str(input_esterel_plan.nodes[edge.source_ids[0]].node_id) + ".start"
                if input_esterel_plan.nodes[edge.source_ids[0]].node_type == 1: # ACTION_END
                    source = "n" + str(input_esterel_plan.nodes[edge.source_ids[0]].node_id-1) + ".end"

                sink = ".zero"
                if input_esterel_plan.nodes[edge.sink_ids[0]].node_type == 0: # ACTION_START
                    sink = "n" + str(input_esterel_plan.nodes[edge.sink_ids[0]].node_id) + ".start"
                if input_esterel_plan.nodes[edge.sink_ids[0]].node_type == 1: # ACTION_END
                    sink = "n" + str(input_esterel_plan.nodes[edge.sink_ids[0]].node_id-1) + ".end"

                stn_plan_file.write("c: " + sink + " - " + source + " in " + bounds + ";\n")
            stn_plan_file.close()


    def paramter_relate_edge(self):
        rospy.loginfo("EMRE SAVAS " + self.STN_plan_path)
        stn_plan_file = open(self.STN_plan_path, 'r')
        for line in stn_plan_file:
            if line.find('[dur_') > 0:
                for key, value in self.dict_stn.items():
                    if value == line[3:(line.find("-")-1)]:
                        #modificatiobs for the robust plan : update the bounds
                        par_edge = str(set(self.output_robust_plan_msg.nodes[0].edges_out).intersection(self.output_robust_plan_msg.nodes[key].edges_in))
                        par_edge_id = int(par_edge[1:(len(par_edge)-1)])
                        self.output_robust_plan_msg.edges[par_edge_id].duration_lower_bound = self.dict_dur_lower[line[(line.find('[')+1):line.find(',')]]
                        self.output_robust_plan_msg.edges[par_edge_id].duration_upper_bound = self.dict_dur_upper[line[(line.find('[')+1):line.find(',')]]


    def final_bound(self,r):
        self.best_rect = {p:v for (p, _), v in r.items()}
        print('yes')

    def serviceCB(self, req):

        if not self.esterel_plan_received:
            rospy.loginfo('KCL: (' + rospy.get_name() + ') No plan received yet!')

        else:
            # call STN python tool
            rospy.loginfo('KCL: (' + rospy.get_name() + ') Calling STN python tool')

            self.best_rect = None
            res = compute_envelope_construct(self.domain_path,self.problem_path,self.STN_plan_path,
                    rectangle_callback = self.final_bound, solver='z3', qelim_name='msat_lw',
                    debug=False, splitting='monolithic', early_forall_elimination=False,
                    compact_encoding=True, bound=1, simplify_effects=True, timeout=self.stn_timeout)
            rospy.loginfo('KCL: (' + rospy.get_name() + ') STNTool terminated')

            # Even if the algorithm is not completed, we might have a valid
            # rectangle stored in self.best_rect!
            if res is None:
                res = self.best_rect

            if res:
                rospy.loginfo('KCL: (' + rospy.get_name() + ') STNTool returned a meaningful rectangle')
                for p, (l, u) in res.items():
                    print(p.name + " in [" + str(l) + ", " + str(u)  + "]")
                    #the upper and lower bound on the edges for parameters
                    self.dict_dur_lower[p.name] = float(l)
                    self.dict_dur_upper[p.name] = float(u)

                    self.output_robust_plan_msg.edges[self.dict_params[p.name]].duration_lower_bound = float(l)
                    self.output_robust_plan_msg.edges[self.dict_params[p.name]].duration_upper_bound = float(u)

                #self.paramter_relate_edge()
                self.publish_robust = True
            else:
                rospy.logwarn("The problem is unsatisfiable!")
        return EmptyResponse()


    def stnCallback(self, msg):
     
        #write msg.data to textfile
        file = open(self.Esterel_plan_path,'w')
        file.write(msg.data)
        file.close()

        Esterel_plan_file = open(self.Esterel_plan_path,'r')

        #number of parameters added to the stn_plan
        n = 0
        #counter on the number of the parameters added
        l = 1
        # esterel nodes dictioanry
        dict_esterel = dict()
        key = 0
        STN_node = 0
        for line in Esterel_plan_file:
            lineSplit = line.split(' ')
            if line[0].isdigit() and line[0]!='0':
                key = int(lineSplit[0][:-1])
                cleanLine = lineSplit[1][7:-1].split('_')
                item = ''
                for word in cleanLine:
                    if word != 'end' and word != 'start':
                        item = item + word +'_'
                dict_esterel[key] = item[:-1]
            elif key > 0:
                endln = ''
                for word1 in lineSplit[0][1:-4].split('"')[0].split(','):
                    endln = endln + word1 + ' '
                dict_esterel[key] = dict_esterel[key]+ ' ' +endln[:-1]
                key = 0

        # map Esterel node to stn node dictionary
        for key in dict_esterel:
            if key % 2 ==1 :
                stn_key_start = 'n' + str(STN_node) + '.start'
                self.dict_stn[key] = stn_key_start
            if key % 2 ==0 :
                stn_key_end = 'n' + str(STN_node) + '.end'
                self.dict_stn[key] = stn_key_end
                STN_node += 1
            self.dict_stn[0] = '.zero'

        STN_node = 0
        stn_plan_file = open(self.STN_plan_path, 'w')
        for key in dict_esterel:
            if key % 2 == 1:
                stn_plan_file.write('durative-action instance n' + str(STN_node) + ' : (' + str(dict_esterel[key]) + ';\n')
                STN_node += 1

        Esterel_plan_file = open(self.Esterel_plan_path,'r')
        for line in Esterel_plan_file:
            if line[0] == '"' :
                lineSplit = line.split('"')
                # parameterizing all the start-end actions
                if int(lineSplit[1]) % 2 ==1 and int(lineSplit[3]) == int(lineSplit[1]) +1 and dict_esterel[int(lineSplit[1])].split('_')[0] == 'goto':
                    default_value= lineSplit[5].split(',')[1].split(']')
                    stn_plan_file.write('parameter dur_' + str(l) + ' default' + str(default_value[0]) + '; \n')
                    stn_plan_file.write('c: ' + str(self.dict_stn[int(lineSplit[3])]) + ' - ' + str(self.dict_stn[int(lineSplit[1])]) + ' in [dur_' + str(l) + ',dur_' + str(l) + '];\n')
                    l+=1

                else:
                    stn_plan_file.write('c: ' + str(self.dict_stn[int(lineSplit[3])]) + ' - ' + str(self.dict_stn[int(lineSplit[1])]) + ' in ' + str(lineSplit[5]) + ';\n')

        stn_plan_file.close()

     # publishing the robutst plan
    def republish_plan(self):

        while not rospy.is_shutdown():
            if self.publish_robust:
                self.pub_robust_plan.publish(self.output_robust_plan_msg)
                self.publish_robust = False
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('robust_envelope_node', anonymous=False)
    robust_envelope_node = RobustEnvelope()
    robust_envelope_node.republish_plan()
