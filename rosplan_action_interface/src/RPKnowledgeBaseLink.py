#!/usr/bin/env python3

from collections import defaultdict

import rospy
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from rosplan_knowledge_msgs.srv import GetDomainPredicateDetailsService, GetDomainPredicateDetailsServiceRequest
from rosplan_knowledge_msgs.srv import GetDomainOperatorDetailsService, GetDomainOperatorDetailsServiceRequest
from rosplan_knowledge_msgs.srv import GetDomainAttributeService
from rosplan_knowledge_msgs.srv import GetAttributeService, GetAttributeServiceRequest
from rosplan_knowledge_msgs.srv import GetInstanceService, GetInstanceServiceRequest
from rosplan_knowledge_msgs.srv import SetNamedBool, SetNamedBoolRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue

class RPKnowledgeBaseLink():

    def __init__(self):

        # initialise clients and publishers
        rospy.wait_for_service('/rosplan_knowledge_base/update_array')
        self.update_kb_srv = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)

        rospy.wait_for_service('/rosplan_knowledge_base/domain/predicate_details')
        self.get_predicates_srv = rospy.ServiceProxy('/rosplan_knowledge_base/domain/predicate_details', GetDomainPredicateDetailsService)

        rospy.wait_for_service('/rosplan_knowledge_base/domain/operator_details')
        self.get_operator_details_srv = rospy.ServiceProxy('/rosplan_knowledge_base/domain/operator_details', GetDomainOperatorDetailsService)

        rospy.wait_for_service('/rosplan_knowledge_base/domain/functions')
        self.get_functions_srv = rospy.ServiceProxy('/rosplan_knowledge_base/domain/functions', GetDomainAttributeService)

        rospy.wait_for_service('/rosplan_knowledge_base/state/instances')
        self.get_instances_srv = rospy.ServiceProxy('/rosplan_knowledge_base/state/instances', GetInstanceService)

        rospy.wait_for_service('/rosplan_knowledge_base/state/propositions')
        self.get_state_propositions_srv = rospy.ServiceProxy('/rosplan_knowledge_base/state/propositions', GetAttributeService)

        rospy.wait_for_service('/rosplan_knowledge_base/state/functions')
        self.get_state_functions_srv = rospy.ServiceProxy('/rosplan_knowledge_base/state/functions', GetAttributeService)

        # predicates are by default not sensed
        self.sensed_predicates = defaultdict(bool)

    # ===================== #
    # action effect methods #
    # ===================== #    

    # effect time variables
    AT_START = 0
    AT_END = 1

    def pack_simple_effect(self, effect, bound_parameters, kus, add_effect):

        # if predicate name is sensed predicate, return
        if self.sensed_predicates[effect.name]:
            return
        
        # check if predicate is now sensed
        pred_details = self.get_kb_predicate_details(effect.name)
        if pred_details != None:        
            self.sensed_predicates[effect.name] = pred_details.is_sensed
            if self.sensed_predicates[effect.name]:
                return

        # add effect to update
        item = self.ground_simple_effect(effect, bound_parameters)
        kus.knowledge.append(item)
        if add_effect:
            kus.update_type.append(kus.ADD_KNOWLEDGE)
        else:
            kus.update_type.append(kus.REMOVE_KNOWLEDGE)

    def kb_apply_action_effects(self, pddl_action_msg, effect_time):

        # request object for update
        kus = KnowledgeUpdateServiceArrayRequest()

        # fetch operator details
        operator_name = pddl_action_msg.name.split(" ")[0]
        operator_details = self.get_kb_operator_details(operator_name)
        if not operator_details:
            rospy.logerr('KCL: ({}) error fetching operator details from KB: {}'.format(rospy.get_name(), operator_name))
            return

        # set up operator parameters for use in effects
        bound_parameters = {}
        for op_parameter in operator_details.formula.typed_parameters:
            found = False
            for i in range(0, len(pddl_action_msg.parameters)):
                if op_parameter.key == pddl_action_msg.parameters[i].key:
                    bound_parameters[pddl_action_msg.parameters[i].key] = pddl_action_msg.parameters[i].value
                    found = True
                    break
            if not found:
                rospy.logerr('KCL: ({}) aborting applying action effect due to missing parameter, missing {}'.format(rospy.get_name(), op_parameter.key))
                return

        kus.knowledge = []
        kus.update_type = []
        if effect_time == self.AT_START:
            # At start add effects
            for eff in operator_details.at_start_add_effects:
                self.pack_simple_effect(eff, bound_parameters, kus, add_effect=True)
            # At start del effects
            for eff in operator_details.at_start_del_effects:
                self.pack_simple_effect(eff, bound_parameters, kus, add_effect=False)

        if effect_time == self.AT_END:
            # At end add effects
            for eff in operator_details.at_end_add_effects:
                self.pack_simple_effect(eff, bound_parameters, kus, add_effect=True)
            # At end del effects
            for eff in operator_details.at_end_del_effects:
                self.pack_simple_effect(eff, bound_parameters, kus, add_effect=False)

        self.update_kb(kus)

    def kb_undo_action_effects(self, pddl_action_msg, effect_time):

        # request object for update
        kus = KnowledgeUpdateServiceArrayRequest()

        # fetch operator details
        operator_name = pddl_action_msg.name.split(" ")[0]
        operator_details = self.get_kb_operator_details(operator_name)
        if not operator_details:
            rospy.logerr('KCL: ({}) error fetching operator details from KB: {}'.format(rospy.get_name(), operator_name))
            return

        # set up operator parameters for use in effects
        bound_parameters = {}
        for op_parameter in operator_details.formula.typed_parameters:
            found = False
            for i in range(0, len(pddl_action_msg.parameters)):
                if op_parameter.key == pddl_action_msg.parameters[i].key:
                    bound_parameters[pddl_action_msg.parameters[i].key] = pddl_action_msg.parameters[i].value
                    found = True
                    break
            if not found:
                rospy.logerr('KCL: ({}) aborting applying action effect due to missing parameter, missing {}'.format(rospy.get_name(), op_parameter.key))
                return

        kus.knowledge = []
        kus.update_type = []
        if effect_time == self.AT_START:
            # At start add effects
            for eff in operator_details.at_start_add_effects:
                self.pack_simple_effect(eff, bound_parameters, kus, add_effect=False)
            # At start del effects
            for eff in operator_details.at_start_del_effects:
                self.pack_simple_effect(eff, bound_parameters, kus, add_effect=True)

        if effect_time == self.AT_END:
            # At end add effects
            for eff in operator_details.at_end_add_effects:
                self.pack_simple_effect(eff, bound_parameters, kus, add_effect=False)
            # At end del effects
            for eff in operator_details.at_end_del_effects:
                self.pack_simple_effect(eff, bound_parameters, kus, add_effect=True)

        self.update_kb(kus)

    def ground_simple_effect(self, effect, bound_parameters):

        # TODO fetch predicate labels

        # bind objects from action message to effect predicate
        item = KnowledgeItem()
        item.knowledge_type = KnowledgeItem.FACT
        item.attribute_name = effect.name
        for j in range(0, len(effect.typed_parameters)):
            # set key (parameter label in predicate) and value (object bound to operator)
            pair = KeyValue()
            pair.key = "TODO THE LABEL"# self.predicates[self.op.at_start_del_effects[i].name].typed_parameters[j].key
            pair.value = bound_parameters[effect.typed_parameters[j].key]
            item.values.append(pair)

        return item

    # ================================== #
    # update service(s) to knowldge base #
    # ================================== #
        
    def update_kb(self, knowledge_update_service_request):
        if len(knowledge_update_service_request.update_type) > 0:
            try:
                self.update_kb_srv.call(knowledge_update_service_request)
            except Exception as e:
                rospy.logerr('KCL: ({}) Failed to update knowledge base: {}'.format(rospy.get_name(), e.message))

    # =============================== #
    # request data from knowldge base #
    # =============================== #

    def get_kb_attribute(self, attribute_name):
        request = GetAttributeServiceRequest(attribute_name)
        try: 
            # call service
            ret = self.get_state_propositions_srv.call(request)
            if len(ret.attributes) > 0:
                return ret.attributes
            return self.get_state_functions_srv.call(request).attributes
        except rospy.ServiceException as exc:
            rospy.logwarn('KCL: ({}) Failed to fetch attribute from knowledge base: {}'.format(rospy.get_name(), str(exc)))
            return None

    def get_kb_operator_details(self, operator_name):
        request = GetDomainOperatorDetailsServiceRequest(operator_name)
        try: 
            # call service
            ret = self.get_operator_details_srv.call(request)
            return ret.op
        except rospy.ServiceException as exc:
            rospy.logwarn('KCL: ({}) Failed to fetch operator from knowledge base: {}'.format(rospy.get_name(), str(exc)))
            return None

    def get_kb_predicate_details(self, predicate_name):
        request = GetDomainPredicateDetailsServiceRequest(predicate_name)
        try:
            # call service
            ret = self.get_predicates_srv.call(request)
        except rospy.ServiceException as exc:
            rospy.logerr('KCL: ({}) error fetching predicate details from KB: {}'.format(rospy.get_name(), str(exc)))
            return None
        return ret


    def get_kb_function_details(self, predicate_name):
        request = GetDomainPredicateDetailsServiceRequest(predicate_name)
        try:
            # call service
            ret = self.get_functions_srv.call(request)
        except rospy.ServiceException as exc:
            rospy.logerr('KCL: ({}) error fetching function details from KB: {}'.format(rospy.get_name(), str(exc)))
            return None
        return ret

    def get_kb_function_parameters(self, func_name):
        try:
            # call service
            functions = self.get_functions_srv.call()
            for i in functions.items:
                if i.name == func_name:
                    return i
        except rospy.ServiceException as exc:
            rospy.logerr('KCL: ({}) error fetching functions from KB: {}'.format(rospy.get_name(), str(exc)))
        return None

    def get_kb_instances(self, type_name):
        request = GetInstanceServiceRequest(type_name)
        try:
            # call service
            ret = self.get_instances_srv.call()
        except rospy.ServiceException as exc:
            rospy.logerr('KCL: ({}) error fetching instances from KB: {}'.format(rospy.get_name(), str(exc)))
            return None
        return ret.instances

