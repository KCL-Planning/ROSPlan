#!/usr/bin/env python

import rospy
import rospkg
import re
import threading
import numpy as np
from threading import Lock
from mongodb_store.message_store import MessageStoreProxy
import imp
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from rosplan_knowledge_msgs.srv import GetDomainPredicateDetailsService, GetDomainPredicateDetailsServiceRequest
from rosplan_knowledge_msgs.srv import GetDomainAttributeService
from rosplan_knowledge_msgs.srv import GetAttributeService, GetAttributeServiceRequest
from rosplan_knowledge_msgs.srv import GetInstanceService, GetInstanceServiceRequest
from rosplan_knowledge_msgs.srv import SetNamedBool, SetNamedBoolRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem
# Todo extend for services? # https://answers.ros.org/question/263005/how-can-i-check-if-a-service-exists-from-python/
# Todo parameters
# FIXME how to define two predicates with the same name?


class RosplanSensing:
    def __init__(self):
        self.mutex = Lock()
        self.srv_mutex = Lock()

        ################################################################################################################
        # Init clients and publishers
        rospy.wait_for_service('/rosplan_knowledge_base/update_array')
        self.update_kb_srv = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
        rospy.wait_for_service('/rosplan_knowledge_base/domain/predicate_details')
        self.get_predicates_srv = rospy.ServiceProxy('/rosplan_knowledge_base/domain/predicate_details', GetDomainPredicateDetailsService)
        rospy.wait_for_service('/rosplan_knowledge_base/domain/functions')
        self.get_functions_srv = rospy.ServiceProxy('/rosplan_knowledge_base/domain/functions', GetDomainAttributeService)
        rospy.wait_for_service('/rosplan_knowledge_base/state/instances')
        self.get_instances_srv = rospy.ServiceProxy('/rosplan_knowledge_base/state/instances', GetInstanceService)
        rospy.wait_for_service('/rosplan_knowledge_base/state/propositions')
        self.get_state_propositions_srv = rospy.ServiceProxy('/rosplan_knowledge_base/state/propositions', GetAttributeService)
        rospy.wait_for_service('/rosplan_knowledge_base/state/functions')
        self.get_state_functions_srv = rospy.ServiceProxy('/rosplan_knowledge_base/state/functions', GetAttributeService)

        set_sensed_predicate_srv = rospy.ServiceProxy('/rosplan_knowledge_base/update_sensed_predicates', SetNamedBool)

        ################################################################################################################
        # Get cfg
        self.cfg_topics = self.cfg_service = {}
        self.functions_path = None
        found_config = False
        if rospy.has_param('~topics'):
            self.cfg_topics = rospy.get_param('~topics')
            found_config = True
        if rospy.has_param('~services'):
            self.cfg_service = rospy.get_param('~services')
            found_config = True
        if rospy.has_param('~functions'):
            self.functions_path = rospy.get_param('~functions')[0]
            regexp = re.compile('\$\(find (.*)\)')
            groups = regexp.match(self.functions_path).groups()
            if len(groups):
                try:
                    ros_pkg_path = rospkg.RosPack().get_path(groups[0])
                    self.functions_path = regexp.sub(ros_pkg_path, self.functions_path)
                except:
                    rospy.logerr('KCL: (RosplanSensing) Error: Package %s was not found! Fix configuration file and retry.' % groups[0])
                    rospy.signal_shutdown('Wrong path in cfg file')
                    return
        if not found_config:
            rospy.logerr('KCL: (RosplanSensing) Error: configuration file is not defined!')
            rospy.signal_shutdown('Config not found')
            return

        # Prepare MessageStoreProxy
        # Todo check if message_Store is present before creating it?
        self.msproxy = MessageStoreProxy()

        ################################################################################################################
        # Load scripts
        if self.functions_path:
            self.scripts = imp.load_source('sensing_scripts', self.functions_path)
            # Declare tools in the scripts module:
            self.scripts.mongodb = self.mongodb
            self.scripts.get_kb_attribute = self.get_kb_attribute
            self.scripts.rospy = rospy

        ################################################################################################################
        # Init variables
        self.sensed_topics = {}
        self.sensed_services = {}
        self.params = {}  # Params defined for each predicate

        ######
        # Subscribe to all the topics
        self.offset = {}  # Offset for reading cfg
        for predicate_name, predicate_info in self.cfg_topics.iteritems():
            params_loaded = self.load_params(predicate_name, predicate_info)  # If parameters found, add 1 to the indexes
            if not params_loaded[0]:
                continue
            offset = self.offset[predicate_name] = int(params_loaded[1])
            if len(predicate_info) < 2+offset:
                rospy.logerr("Error: Wrong configuration file for predicate %s" % predicate_name)
                continue
            msg_type = predicate_info[offset+1]
            self.import_msg(msg_type)
            rospy.Subscriber(predicate_info[offset+0], eval(msg_type[msg_type.rfind('/')+1:]), self.subs_callback, predicate_name)
            # self.sensed_topics[predicate_name] = (None, False)
            try:  # Update KB to inform about the sensed predicates
                sensed_srv_req = SetNamedBoolRequest()
                sensed_srv_req.name = predicate_name
                sensed_srv_req.value = True
                set_sensed_predicate_srv.call(sensed_srv_req)
            except Exception as e:
                rospy.logerr('KCL: (RosplanSensing) Could not update sensing information in Knowledge Base for proposition %s',
                             predicate_name)
            rospy.loginfo('KCL: (RosplanSensing) Predicate %s: Subscribed to topic %s of type %s', predicate_name,
                          predicate_info[offset+0], msg_type)

        ############
        # Create clients for all the services
        self.service_clients = []
        self.service_type_names = []
        self.service_predicate_names = []
        self.last_called_time = []
        self.time_between_calls = []
        self.request_src = []
        self.response_process_src = []
        for predicate_name, predicate_info in self.cfg_service.iteritems():
            params_loaded = self.load_params(predicate_name, predicate_info)  # If parameters found, add 1 to the indexes
            if not params_loaded[0]:
                continue
            offset = self.offset[predicate_name] = int(params_loaded[1])
            if len(predicate_info) < 2+offset:
                rospy.logerr("Error: Wrong configuration file for predicate %s" % predicate_name)
                continue
            srv_type = predicate_info[offset+1]
            srv_typename = self.import_srv(srv_type)
            self.service_type_names.append(srv_typename)
            self.service_clients.append(rospy.ServiceProxy(predicate_info[offset+0], eval(srv_typename)))
            self.service_predicate_names.append(predicate_name)
            self.last_called_time.append(rospy.Time(0))
            self.time_between_calls.append(predicate_info[offset+2])

            # Gets the request creation
            request_offset = 1
            if predicate_info[offset+3].find('req ') == 0:
                self.request_src.append(predicate_info[offset+3].replace('req ', '', 1))
            elif predicate_info[offset+3].find('request ') == 0:
                self.request_src.append(predicate_info[offset+3].replace('request ', '', 1))
            else:
                self.request_src.append(None)
                request_offset = 0

            # Gets the string processing
            if len(predicate_info) == offset+request_offset+4:
                self.response_process_src.append(predicate_info[offset+request_offset+3])
            else:
                self.response_process_src.append(None)


            try:  # Update KB to inform about the sensed predicates
                sensed_srv_req = SetNamedBoolRequest()
                sensed_srv_req.name = predicate_name
                sensed_srv_req.value = True
                set_sensed_predicate_srv.call(sensed_srv_req)
            except Exception as e:
                rospy.logerr('KCL: (RosplanSensing) Could not update sensing information in Knowledge Base for proposition %s',
                             predicate_name)
            rospy.loginfo('KCL: (RosplanSensing) Predicate %s: Client for service %s of type %s is ready', predicate_name,
                          predicate_info[offset+0], srv_type)

    # Returns (bool, bool), first tells if the parameters could be loaded, second if parameters were loaded from config file and false if they were taken directly from the kb
    def load_params(self, predicate_name, predicate_info):
        # Check if predicate
        kb_info = None
        try:
            kb_info = self.get_predicates_srv.call(GetDomainPredicateDetailsServiceRequest(predicate_name)).predicate
        except Exception as e:
            kb_info = self.get_function_params(predicate_name)
            if not kb_info:
                rospy.logerr("KCL: (RosplanSensing) Could not find predicate or function %s" % predicate_name)
                return (False, False)

        # Obtain all the instances for each parameter

        kb_params = []
        for p in kb_info.typed_parameters:
            instances = self.get_instances_srv.call(GetInstanceServiceRequest(p.value)).instances
            kb_params.append(instances)

        if predicate_info[0].startswith('params'):
            params = predicate_info[0].split()
            params.pop(0)  # Remove the 'params' string
            if len(kb_params) != len(params):
                rospy.logerr("KCL: (RosplanSensing) Parameters defined for predicate %s don't match the knowledge base" % predicate_name)
                rospy.signal_shutdown('Wrong cfg file parameters definition')
                return (False, True)
            # Check params
            wildcard = False
            for i, p in enumerate(params):
                if p != '*' and p != '_':
                    if p in kb_params[i]:
                        kb_params[i] = [p]
                    else:
                        rospy.logerr('KCL: (RosplanSensing) Unknown parameter instance "%s" of type "%s" for predicate "%s"',
                                     p, kb_info.typed_parameters[i].value, predicate_name)
                        rospy.signal_shutdown('Wrong cfg file parameters definition')
                        return (False, True)
                else:
                    wildcard = True

            # If the params are fully instantiated we store them as a list, else it will be a matrix with a list of
            #  instances per parameter
            self.params[predicate_name] = kb_params if wildcard else params
            return (True, True)
        else:
            self.params[predicate_name] = kb_params
            return (True, False)

    def get_function_params(self, func_name):
        functions = self.get_functions_srv.call()
        for i in functions.items:
            if i.name == func_name:
                return i
        return None

    def subs_callback(self, msg, pred_name):
        offset = self.offset[pred_name]
        if len(self.cfg_topics[pred_name]) == 3+offset:
            python_string = self.cfg_topics[pred_name][2+offset]
        else:  # Call the method from the scripts.py file
            if not pred_name in dir(self.scripts):
                rospy.logerr('KCL: (RosplanSensing) Predicate "%s" does not have either a function or processing information' % pred_name)
                return None
            python_string = "self.scripts." + pred_name + "(msg, self.params[pred_name])"

        result = eval(python_string, globals(), locals())
        changed = False

        if type(result) is list:
            for params, val in result:
                self.mutex.acquire(True)  # ROS subscribers are multithreaded in Python
                try:
                    if type(val) is bool:
                        changed = self.sensed_topics[pred_name + ':' + params][0] ^ val
                    else:
                        changed = self.sensed_topics[pred_name + ':' + params][0] != val
                except:  # If hasn't been added yet just ignore it
                    changed = True
                self.sensed_topics[pred_name + ':' + params] = (val, changed)
                self.mutex.release()
        else:
            params = reduce(lambda r, x: r + ':' + x, self.params[pred_name])
            if type(params) == list:
                rospy.logerr('KCL: (RosplanSensing) Predicate "%s" needs to have all the parameters defined and fully instantiated' % pred_name)
                rospy.signal_shutdown('Wrong cfg params')
                return None
            self.mutex.acquire(True)  # ROS subscribers are multithreaded in Python
            try:
                if type(result) is bool:
                    changed = self.sensed_topics[pred_name + ':' + params][0] ^ result
                else:
                    changed = self.sensed_topics[pred_name + ':' + params][0] != result
            except:  # If hasn't been added yet just ignore it
                changed = True
            self.sensed_topics[pred_name + ':' + params] = (result, changed)
            self.mutex.release()

    # Import a ros msg type of type pkg_name/MessageName
    def import_msg(self, ros_msg_string):
        # msg_string will be something like std_msgs/String -> convert it to from std_msgs.msg import String
        i = ros_msg_string.find('/')
        pkg_name = ros_msg_string[:i]
        msg_name = ros_msg_string[i+1:]
        exec('from ' + pkg_name + ".msg import " + msg_name, globals())

    # Import a ros srv type of type pkg_name/MessageName
    def import_srv(self, ros_srv_string):
        # srv str will be something like std_msgs/String -> convert it to from std_msgs.srv import String StringRequest
        i = ros_srv_string.find('/')
        pkg_name = ros_srv_string[:i]
        srv_name = ros_srv_string[i + 1:]
        exec ('from ' + pkg_name + ".srv import " + srv_name + ", " + srv_name + "Request", globals())
        return srv_name

    # To be run in its own thread
    def call_services(self):
        # TODO time handling and parameters and outer loop
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            for i in xrange(len(self.service_clients)):
                now = rospy.Time.now()
                if (now-self.last_called_time[i]) < rospy.Duration(self.time_between_calls[i]):
                    continue

                pred_name = self.service_predicate_names[i]
                # Get request
                if self.request_src[i]:
                    python_string = self.request_src[i]
                else:  # Call the method from the scripts.py file
                    if not ('req_' + pred_name) in dir(self.scripts):
                        rospy.logerr(
                            'KCL: (RosplanSensing) Predicate "%s" does not have either a Request creation method' % pred_name)
                        continue
                    python_string = "self.scripts." + 'req_' + pred_name + "()"

                req = eval(python_string, globals(), locals())

                try:
                    res = self.service_clients[i].call(req)
                    self.last_called_time[i] = now
                except Exception as e:
                    rospy.logerr("KCL (SensingInterface) Failed to call service for predicate %s base: %s" % (pred_name, e.message))
                    continue

                # Process response
                if self.request_src[i]:
                    python_string = self.request_src[i]
                else:  # Call the method from the scripts.py file
                    if not pred_name in dir(self.scripts):
                        rospy.logerr('KCL: (RosplanSensing) Predicate "%s" does not have either a function or processing information' %
                            pred_name)
                        continue
                    python_string = "self.scripts." + pred_name + "(res, self.params[pred_name])"

                result = eval(python_string, globals(), locals())
                changed = False

                if type(result) is list:
                    for params, val in result:
                        self.srv_mutex.acquire(True)  # ROS subscribers are multithreaded in Python
                        try:
                            if type(val) is bool:
                                changed = self.sensed_services[pred_name + ':' + params][0] ^ val
                            else:
                                changed = self.sensed_services[pred_name + ':' + params][0] != val
                        except:  # If hasn't been added yet just ignore it
                            changed = True
                        self.sensed_services[pred_name + ':' + params] = (val, changed)
                        self.srv_mutex.release()
                else:
                    params = reduce(lambda r, x: r + ':' + x, self.params[pred_name])
                    if type(params) == list:
                        rospy.logerr(
                            'KCL: (RosplanSensing) Predicate "%s" needs to have all the parameters defined and fully instantiated' % pred_name)
                        rospy.signal_shutdown('Wrong cfg params')
                        return None
                    self.srv_mutex.acquire(True)  # ROS subscribers are multithreaded in Python
                    try:
                        if type(result) is bool:
                            changed = self.sensed_services[pred_name + ':' + params][0] ^ result
                        else:
                            changed = self.sensed_services[pred_name + ':' + params][0] != result
                    except:  # If hasn't been added yet just ignore it
                        changed = True
                    self.sensed_services[pred_name + ':' + params] = (result, changed)
                    self.srv_mutex.release()

            r.sleep()
        return None  # Finish thread

    def update_kb(self):
        kus = KnowledgeUpdateServiceArrayRequest()
        # Get info from KB functions and propositions (to know type of variable)
        # Fill update
        self.mutex.acquire(True)
        sensed_predicates = self.sensed_topics.copy()
        self.mutex.release()
        self.srv_mutex.acquire(True)
        sensed_predicates.update(self.sensed_services.copy())
        self.srv_mutex.release()
        for predicate, (val, changed) in sensed_predicates.iteritems():
            if not changed:
                continue

            if predicate in self.sensed_topics:
                self.mutex.acquire(True)
                self.sensed_topics[predicate] = (self.sensed_topics[predicate][0], False)  # Set to not changed
                self.mutex.release()
            else:
                self.srv_mutex.acquire(True)
                self.sensed_services[predicate] = (self.sensed_services[predicate][0], False)  # Set to not changed
                self.srv_mutex.release()
            predicate_info = predicate.split(':')
            ki = KnowledgeItem()
            ki.attribute_name = predicate_info.pop(0)
            if type(val) is bool:
                ki.knowledge_type = ki.FACT
                ki.is_negative = not val
            else:
                ki.knowledge_type = ki.FUNCTION
                ki.function_value = val

            # TODO what to do if no parameters specified? iterate over all the instantiated parameters and add it?
            for i, param in enumerate(predicate_info):
                kv = KeyValue()
                kv.key = 'p' + str(i)
                kv.value = param
                ki.values.append(kv)

            kus.update_type += np.array(kus.ADD_KNOWLEDGE).tostring()
            kus.knowledge.append(ki)

        # Update the KB with the full array
        if len(kus.update_type) > 0:
            try:
                self.update_kb_srv.call(kus)
            except Exception as e:
                rospy.logerr("KCL (SensingInterface) Failed to update knowledge base: %s" % e.message)

    # Returns the query to the mongodb messageproxy object
    def mongodb(self, query, ret_type):
        return self.msproxy.query_named(query, ret_type)

    def get_kb_attribute(self, attribute_name):
        request = GetAttributeServiceRequest(attribute_name)
        ret = self.get_state_propositions_srv.call(request)
        if len(ret.attributes) > 0:
            return ret.attributes
        return self.get_state_functions_srv.call(request).attributes


if __name__ == "__main__":
    rospy.init_node('rosplan_sensing_interface', anonymous=False)
    rps = RosplanSensing()
    t = threading.Thread(target=RosplanSensing.call_services, args=(rps,))

    # main loop
    rate = rospy.Rate(10)  # 10hz
    t.start()
    while not rospy.is_shutdown():
        # Note: spin is not needed in python as the callback methods are run in a different thread
        rps.update_kb()
        rate.sleep()
    t.join()
