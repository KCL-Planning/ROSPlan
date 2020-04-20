#!/usr/bin/env python

import rospy, copy
from rosplan_knowledge_msgs.msg import DomainFormula, DomainOperator, KnowledgeItem
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback
from rosplan_knowledge_msgs.srv import GetDomainOperatorDetailsService, GetDomainPredicateDetailsService
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceRequest, KnowledgeUpdateServiceArray
from diagnostic_msgs.msg import KeyValue

class RPActionInterface(object):
    def __init__(self):
        self.params = DomainFormula()
        self.op = DomainOperator()
        self.predicates = {} # str, rosplan_knowledge_msgs.DomainFormula dic
        self.sensed_predicates = {} # str, rosplan_knowledge_msgs.DomainFormula dic
        self.action_feedback_pub = None
        self.update_knowledge_client = None

    def dispatchCallback(self, msg):
        # check action name
        if msg.name == 'cancel_action':
            action_cancelled = True
            return None
        if msg.name != self.params.name:
            return None
        rospy.loginfo('KCL: ({}) action received'.format(self.params.name))

        action_cancelled = False

        # check PDDL parameters
        self.params.typed_parameters
        boundParameters = {}
        found = []
        for i in range(0, len(self.params.typed_parameters)):
            found.append(False)
        for j in range(0, len(self.params.typed_parameters)):
            for i in range(0, len(msg.parameters)):
                if self.params.typed_parameters[j].key == msg.parameters[i].key:
                    boundParameters[msg.parameters[i].key] = msg.parameters[i].value
                    found[j] = True
                    break
            if not found[j]:
                rospy.loginfo('KCL: ({}) aborting action dispatch malformed parameters, missing {}'.format(self.params.name, self.params.typed_parameters[j].key))
                return None

        # send feedback (enabled)
        fb = ActionFeedback()
        fb.action_id = msg.action_id
        fb.status = 'action enabled'
        self.action_feedback_pub.publish(fb)

        # update knowledge base
        updatePredSrv = [[] for i in range(2)] # make list of lists

        # simple START del effects
        for i in range(0, len(self.op.at_start_del_effects)):
            if self.op.at_start_del_effects[i].name in self.sensed_predicates:
                continue # sensed predicate

            item = KnowledgeItem()
            item.knowledge_type = KnowledgeItem.FACT
            item.attribute_name = self.op.at_start_del_effects[i].name
            pair = KeyValue()
            for j in range(0, len(self.op.at_start_del_effects[i].typed_parameters)):
                pair.key = self.predicates[self.op.at_start_del_effects[i].name].typed_parameters[j].key
                pair.value = boundParameters[self.op.at_start_del_effects[i].typed_parameters[j].key]
                item.values.append(copy.deepcopy(pair))
            updatePredSrv[0].append(KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE)
            updatePredSrv[1].append(item)

        # simple START add effects
        for i in range(0, len(self.op.at_start_add_effects)):
            if self.op.at_start_add_effects[i].name in self.sensed_predicates:
                continue # sensed predicate

            item = KnowledgeItem()
            item.knowledge_type = KnowledgeItem.FACT
            item.attribute_name = self.op.at_start_add_effects[i].name
            pair = KeyValue()
            for j in range(0, len(self.op.at_start_add_effects[i].typed_parameters)):
                pair.key = self.predicates[self.op.at_start_add_effects[i].name].typed_parameters[j].key
                pair.value = boundParameters[self.op.at_start_add_effects[i].typed_parameters[j].key]
                item.values.append(copy.deepcopy(pair))
            updatePredSrv[0].append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
            updatePredSrv[1].append(item)

        if (len(updatePredSrv[1]) > 0) and (not self.update_knowledge_client(updatePredSrv[0], updatePredSrv[1])):
            rospy.loginfo('KCL: ({}) failed to update PDDL model in knowledge base'.format(self.params.name))

        # call concrete implementation
        action_success = self.concreteCallback(msg)
        if action_cancelled:
            action_success = False
            rospy.loginfo('KCL: ({}) an old action that was cancelled is stopping now'.format(self.params.name))
            return None

        if action_success:
            rospy.loginfo('KCL: ({}) action completed successfully'.format(self.params.name))

            # update knowledge base
            updatePredSrv = [[] for i in range(2)] # make list of lists

            # simple END del effects
            for i in range(0, len(self.op.at_end_del_effects)):
                if self.op.at_end_del_effects[i].name in self.sensed_predicates:
                    continue # sensed predicate

                item = KnowledgeItem()
                item.knowledge_type = KnowledgeItem.FACT
                item.attribute_name = self.op.at_end_del_effects[i].name
                pair = KeyValue()
                for j in range(0, len(self.op.at_end_del_effects[i].typed_parameters)):
                    pair.key = self.predicates[self.op.at_end_del_effects[i].name].typed_parameters[j].key
                    pair.value = boundParameters[self.op.at_end_del_effects[i].typed_parameters[j].key]
                    item.values.append(copy.deepcopy(pair))
                updatePredSrv[0].append(KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE)
                updatePredSrv[1].append(item)

            # simple END add effects
            for i in range(0, len(self.op.at_end_add_effects)):
                if self.op.at_end_add_effects[i].name in self.sensed_predicates:
                    continue # sensed predicate

                item = KnowledgeItem()
                item.knowledge_type = KnowledgeItem.FACT
                item.attribute_name = self.op.at_end_add_effects[i].name
                pair = KeyValue()
                for j in range(0, len(self.op.at_end_add_effects[i].typed_parameters)):
                    pair.key = self.predicates[self.op.at_end_add_effects[i].name].typed_parameters[j].key
                    pair.value = boundParameters[self.op.at_end_add_effects[i].typed_parameters[j].key]
                    item.values.append(copy.deepcopy(pair))
                updatePredSrv[0].append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
                updatePredSrv[1].append(item)

            if (len(updatePredSrv[1]) > 0) and (not self.update_knowledge_client(updatePredSrv[0], updatePredSrv[1])):
                rospy.loginfo('KCL: ({}) failed to update PDDL model in knowledge base'.format(self.params.name))

            # publish feedback (achieved)
            fb.status = 'action achieved'
            self.action_feedback_pub.publish(fb)
        else:
            # publish feedback (failed)
            fb.status = 'action failed'
            self.action_feedback_pub.publish(fb)

    def runActionInterface(self):
        self.params.name = rospy.get_param('~pddl_action_name', None)

        # knowledge base services
        kb = rospy.get_param('~knowledge_base', 'knowledge_base')

        # fetch action params
        ss = '/' + kb + '/domain/operator_details'
        rospy.wait_for_service(ss, 20)
        client = rospy.ServiceProxy(ss, GetDomainOperatorDetailsService)
        try:
            resp = client(self.params.name)
            self.params = resp.op.formula
            self.op = resp.op
        except:
            rospy.logerr('KCL: (RPActionInterface) could not call Knowledge Base for operator details, {}'.format(self.params.name))
            return None

        # collect predicates from operator description
        predicateNames =[]

        # effects
        for eff in self.op.at_start_add_effects:
            predicateNames.append(eff.name)

        for eff in self.op.at_start_add_effects:
            predicateNames.append(eff.name)

        for eff in self.op.at_start_del_effects:
            predicateNames.append(eff.name)

        for eff in self.op.at_end_add_effects:
            predicateNames.append(eff.name)

        for eff in self.op.at_end_del_effects:
            predicateNames.append(eff.name)

        # simple conditions
        for cond in self.op.at_start_simple_condition:
            predicateNames.append(cond.name)

        for cond in self.op.over_all_simple_condition:
            predicateNames.append(cond.name)

        for cond in self.op.at_end_simple_condition:
            predicateNames.append(cond.name)

        # negative conditions
        for ncond in self.op.at_start_neg_condition:
            predicateNames.append(ncond.name)

        for ncond in self.op.over_all_neg_condition:
            predicateNames.append(ncond.name)

        for ncond in self.op.at_end_neg_condition:
            predicateNames.append(ncond.name)

        # fetch and store predicate details
        ss = '/' + kb + '/domain/predicate_details'
        rospy.wait_for_service(ss, 20)
        predClient = rospy.ServiceProxy(ss, GetDomainPredicateDetailsService)
        for predicateName in predicateNames:
            if predicateName in self.predicates:
                continue
            if predicateName == '=' or predicateName == '>' or predicateName == '<' or predicateName == '>=' or predicateName == '<=':
                continue
            try:
                resp = predClient(predicateName)
                if resp.is_sensed:
                    self.sensed_predicates[predicateName] = resp.predicate
                else:
                    self.predicates[predicateName] = resp.predicate
            except:
                rospy.logerr('KCL: (RPActionInterface) could not call Knowledge Base for predicate details, {}'.format(self.params.name))
                return None

        # create PDDL info publisher
        ss = '/' + kb + '/pddl_action_parameters'
        pddl_action_parameters_pub = rospy.Publisher(ss, DomainFormula, queue_size=10)

        # create the action feedback publisher
        aft = rospy.get_param('~action_feedback_topic', 'default_feedback_topic')
        self.action_feedback_pub = rospy.Publisher(aft, ActionFeedback, queue_size=10)

        # knowledge interface
        ss = '/' + kb + '/update_array'
        rospy.wait_for_service(ss, 20)
        self.update_knowledge_client = rospy.ServiceProxy(ss, KnowledgeUpdateServiceArray)

        # listen for action dispatch
        adt = rospy.get_param('~action_dispatch_topic', 'default_dispatch_topic')
        rospy.Subscriber(adt, ActionDispatch, self.dispatchCallback, queue_size=1)
        rospy.loginfo('KCL: ({}) Ready to receive'.format(self.params.name))

        # loop
        loopRate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            pddl_action_parameters_pub.publish(self.params)
            loopRate.sleep()
