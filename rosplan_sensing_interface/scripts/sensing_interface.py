#!/usr/bin/env python3

import rospy
import rospkg
import re
import threading
import numpy as np
from threading import Lock
import importlib
from functools import reduce
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from rosplan_knowledge_msgs.srv import GetDomainPredicateDetailsService, GetDomainPredicateDetailsServiceRequest
from rosplan_knowledge_msgs.srv import GetDomainAttributeService
from rosplan_knowledge_msgs.srv import GetAttributeService, GetAttributeServiceRequest
from rosplan_knowledge_msgs.srv import GetInstanceService, GetInstanceServiceRequest
from rosplan_knowledge_msgs.srv import SetNamedBool, SetNamedBoolRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem, StatusUpdate
from diagnostic_msgs.msg import KeyValue


class RosplanSensing:
    def __init__(self):
        self.mutex = Lock()
        self.srv_mutex = Lock()
        self.dump_cache_mutex = Lock()

        ################################################################################################################
        self.knowledge_base = '/rosplan_knowledge_base'
        if rospy.has_param('~knowledge_base'):
            self.knowledge_base = rospy.get_param('~knowledge_base')

        # Init clients and publishers
        rospy.wait_for_service(self.knowledge_base + '/update_array')
        self.update_kb_srv = rospy.ServiceProxy(self.knowledge_base + '/update_array', KnowledgeUpdateServiceArray)
        rospy.wait_for_service(self.knowledge_base + '/domain/predicate_details')
        self.get_predicates_srv = rospy.ServiceProxy(self.knowledge_base + '/domain/predicate_details', GetDomainPredicateDetailsService)
        rospy.wait_for_service(self.knowledge_base + '/domain/functions')
        self.get_functions_srv = rospy.ServiceProxy(self.knowledge_base + '/domain/functions', GetDomainAttributeService)
        rospy.wait_for_service(self.knowledge_base + '/state/instances')
        self.get_instances_srv = rospy.ServiceProxy(self.knowledge_base + '/state/instances', GetInstanceService)
        rospy.wait_for_service(self.knowledge_base + '/state/propositions')
        self.get_state_propositions_srv = rospy.ServiceProxy(self.knowledge_base + '/state/propositions', GetAttributeService)
        rospy.wait_for_service(self.knowledge_base + '/state/functions')
        self.get_state_functions_srv = rospy.ServiceProxy(self.knowledge_base + '/state/functions', GetAttributeService)

        self.set_sensed_predicate_srv = rospy.ServiceProxy(self.knowledge_base + '/update_sensed_predicates', SetNamedBool)

        self.kb_update_status_subs = rospy.Subscriber(self.knowledge_base + '/status/update', StatusUpdate, self.kb_update_status)

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


        ################################################################################################################
        # Load scripts
        self.scripts = None
        if self.functions_path:
            self.scripts = importlib.machinery.SourceFileLoader('sensing_scripts', self.functions_path).load_module()
            # Declare tools in the scripts module:
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
        for predicate_name, predicate_info in self.cfg_topics.items():
            if type(predicate_info) is list:  # We have nested elements in the predicate
                for i, pi in enumerate(predicate_info):
                    pi['sub_idx'] = i
                    subscribed = self.subscribe_topic(predicate_name, pi)
                    if not subscribed:
                        rospy.loginfo('Could not subscribe for predicate ' + predicate_name + ' and config: ' + str(pi))
                        continue
            else:
                predicate_info['sub_idx'] = 0  # As we don't have nested elements in this predicate
                subscribed = self.subscribe_topic(predicate_name, predicate_info)
                if not subscribed:
                    rospy.loginfo('Could not subscribe for predicate ' + predicate_name + ' and config: ' + str(predicate_info))

        ############
        # Create clients for all the services
        self.service_clients = []
        self.service_names = []
        self.service_type_names = []
        self.service_predicate_names = []
        self.last_called_time = []
        self.time_between_calls = []
        self.request_src = []
        self.response_process_src = []
        self.clients_sub_idx = []
        for predicate_name, predicate_info in self.cfg_service.items():
            if type(predicate_info) is list:
                for i, pi in enumerate(predicate_info):
                    pi['sub_idx'] = i
                    client_created = self.create_service_client(predicate_name, pi)
                    if not client_created:
                        rospy.loginfo('Could not create client for predicate ' + predicate_name + ' and config: ' + str(pi))
                        continue
            else:
                predicate_info['sub_idx'] = 0  # As we don't have nested elements in this predicate
                client_created = self.create_service_client(predicate_name, predicate_info)
                if not client_created:
                    rospy.loginfo('Could not create client for predicate ' + predicate_name + ' and config: ' + str(predicate_info))
                    continue

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

        if predicate_name not in self.params:  # Prepare dictionary
            self.params[predicate_name] = {}

        # Obtain all the instances for each parameter

        kb_params = []
        for p in kb_info.typed_parameters:
            instances = self.get_instances_srv.call(GetInstanceServiceRequest(p.value, True, True)).instances
            kb_params.append(instances)

        if 'params' in predicate_info:
            params = predicate_info['params']
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
            self.params[predicate_name][predicate_info['sub_idx']] = kb_params if wildcard else params
            return (True, True)
        else:
            self.params[predicate_name][predicate_info['sub_idx']] = kb_params
            return (True, False)

    def subscribe_topic(self, predicate_name, predicate_info):
        params_loaded = self.load_params(predicate_name, predicate_info)  # If parameters found, add 1 to the indexes
        if not params_loaded[0]:
            return False
        if len(predicate_info) < 2 + int(params_loaded[1]):
            rospy.logerr("Error: Wrong configuration file for predicate %s" % predicate_name)
            return False
        try:
            msg_type = predicate_info['msg_type']
        except KeyError:
            rospy.logerr("Error: msg_type was not specified for predicate %s" % predicate_name)
            return False
        self.import_msg(msg_type)
        try:
            if isinstance(predicate_info['topic'], list):
                for topic in predicate_info['topic']:
                    pred_info = predicate_info.copy()
                    pred_info['publisher'] = topic
                    rospy.Subscriber(str(topic), eval(msg_type[msg_type.rfind('/') + 1:]), self.subs_callback, (predicate_name, pred_info))
            else:
                rospy.Subscriber(predicate_info['topic'], eval(msg_type[msg_type.rfind('/') + 1:]), self.subs_callback, (predicate_name, predicate_info))
        except KeyError:
            rospy.logerr("Error: topic was not specified for predicate %s" % predicate_name)
            return False
        # self.sensed_topics[predicate_name] = (None, False)
        try:  # Update KB to inform about the sensed predicates
            sensed_srv_req = SetNamedBoolRequest()
            sensed_srv_req.name = predicate_name
            sensed_srv_req.value = True
            self.set_sensed_predicate_srv.call(sensed_srv_req)
        except Exception as e:
            rospy.logerr(
                'KCL: (RosplanSensing) Could not update sensing information in Knowledge Base for proposition %s',
                predicate_name)
        rospy.loginfo('KCL: (RosplanSensing) Predicate %s: Subscribed to topic %s of type %s', predicate_name,
                      predicate_info['topic'], msg_type)
        return True

    def create_service_client(self, predicate_name, predicate_info):
        params_loaded = self.load_params(predicate_name, predicate_info)  # If parameters found, add 1 to the indexes
        if not params_loaded[0]:
            return False
        if len(predicate_info) < 2 + int(params_loaded[1]):
            rospy.logerr("Error: Wrong configuration file for predicate %s" % predicate_name)
            return False
        try:
            srv_type = predicate_info['srv_type']
        except KeyError:
            rospy.logerr("Error: service was not specified for predicate %s" % predicate_name)
            return False
        srv_typename = self.import_srv(srv_type)
        self.service_type_names.append(srv_typename)
        self.clients_sub_idx.append(predicate_info['sub_idx'])
        try:
            self.service_clients.append(rospy.ServiceProxy(predicate_info['service'], eval(srv_typename)))
            self.service_names.append(predicate_info['service'])
        except KeyError:
            rospy.logerr("Error: service was not specified for predicate %s" % predicate_name)
            return False
        self.service_predicate_names.append(predicate_name)
        self.last_called_time.append(rospy.Time(0))

        try:
            self.time_between_calls.append(predicate_info['time_between_calls'])
        except:
            rospy.logerr("Error: time_between_calls was not specified for predicate %s" % predicate_name)
            return False

        # Gets the request creation
        if 'request' in predicate_info:
            self.request_src.append(predicate_info['request'])
        else:
            self.request_src.append(None)

        # Gets the result processing
        if 'operation' in predicate_info:
            self.response_process_src.append(predicate_info['operation'])
        else:
            self.response_process_src.append(None)

        try:  # Update KB to inform about the sensed predicates
            sensed_srv_req = SetNamedBoolRequest()
            sensed_srv_req.name = predicate_name
            sensed_srv_req.value = True
            self.set_sensed_predicate_srv.call(sensed_srv_req)
        except Exception as e:
            rospy.logerr(
                'KCL: (RosplanSensing) Could not update sensing information in Knowledge Base for proposition %s',
                predicate_name)
        rospy.loginfo('KCL: (RosplanSensing) Predicate %s: Client for service %s of type %s is ready', predicate_name,
                      predicate_info['service'], srv_type)
        return True

    def get_function_params(self, func_name):
        functions = self.get_functions_srv.call()
        for i in functions.items:
            if i.name == func_name:
                return i
        return None

    def subs_callback(self, msg, predicate):
        pred_name, pred_info = predicate
        if 'operation' in pred_info:  # pred_info is of type self.cfg_topics[pred_name]
            python_string = pred_info['operation']
        elif self.scripts:  # Call the method from the scripts.py file
            if not pred_name in dir(self.scripts):
                rospy.logerr('KCL: (RosplanSensing) Predicate "%s" does not have either a function or processing information' % pred_name)
                return None
            if 'publisher' in pred_info:
                python_string = "self.scripts." + pred_name + "(msg, self.params[pred_name][pred_info['sub_idx']], pred_info['publisher'])"
            else:
                python_string = "self.scripts." + pred_name + "(msg, self.params[pred_name][pred_info['sub_idx']])"
        else:
            rospy.logerr('KCL: (RosplanSensing) Predicate "%s" does not have either a function or processing information' % pred_name)
            return None

        self.mutex.acquire(True)
        result = eval(python_string, globals(), locals())
        self.mutex.release()
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
                if changed:
                    self.sensed_topics[pred_name + ':' + params] = (val, changed, False)
                self.mutex.release() 
        else:
            params = reduce(lambda r, x: r + ':' + x, self.params[pred_name][pred_info['sub_idx']])
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
            if changed:
                self.sensed_topics[pred_name + ':' + params] = (result, changed, False)
            self.mutex.release()

    # Import a ros msg type of type pkg_name/MessageName
    def import_msg(self, ros_msg_string):
        # msg_string will be something like std_msgs/String -> convert it to from std_msgs.msg import String
        i = ros_msg_string.find('/')
        pkg_name = ros_msg_string[:i]
        msg_name = ros_msg_string[i+1:]
        exec('from ' + pkg_name + ".msg import " + msg_name, globals())
        if self.scripts:
            exec('self.scripts.' + msg_name + " = " + msg_name, globals(), locals())
        return msg_name

    # Import a ros srv type of type pkg_name/MessageName
    def import_srv(self, ros_srv_string):
        # srv str will be something like std_msgs/String -> convert it to from std_msgs.srv import String StringRequest
        i = ros_srv_string.find('/')
        pkg_name = ros_srv_string[:i]
        srv_name = ros_srv_string[i + 1:]
        exec ('from ' + pkg_name + ".srv import " + srv_name + ", " + srv_name + "Request", globals())
        if self.scripts:
            exec ('self.scripts.' + srv_name + " = " + srv_name, globals(), locals())
            exec ('self.scripts.' + srv_name + "Request = " + srv_name + "Request", globals(), locals())
        return srv_name

    # To be run in its own thread
    def call_services(self):
        rospy.loginfo("Waiting for services to become available...")
        for i in range(len(self.service_names)):
            rospy.loginfo('   Waiting for %s', self.service_names[i])
            rospy.wait_for_service(self.service_names[i], 20)
        rospy.loginfo('All services are ready.')

        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            for i in range(len(self.service_clients)):
                now = rospy.Time.now()
                if (now-self.last_called_time[i]) < rospy.Duration(self.time_between_calls[i]):
                    continue

                pred_name = self.service_predicate_names[i]
                # Get request
                if self.request_src[i]:
                    python_string = self.request_src[i]
                elif self.scripts:  # Call the method from the scripts.py file
                    if not ('req_' + pred_name) in dir(self.scripts):
                        rospy.logerr('KCL: (RosplanSensing) Predicate "%s" does not have either a Request creation method' % pred_name)
                        continue
                    python_string = "self.scripts." + 'req_' + pred_name + "()"
                else:
                    rospy.logerr('KCL: (RosplanSensing) Predicate "%s" does not have either a Request creation method' % pred_name)
                    continue

                req = eval(python_string, globals(), locals())

                try:
                    res = self.service_clients[i].call(req)
                    self.last_called_time[i] = now
                except Exception as e:
                    rospy.logerr("KCL (SensingInterface) Failed to call service for predicate %s base: %s" % (pred_name, e.message))
                    continue

                # Process response
                if self.response_process_src[i]:
                    python_string = self.response_process_src[i]
                elif self.scripts:  # Call the method from the scripts.py file
                    if not pred_name in dir(self.scripts):
                        rospy.logerr('KCL: (RosplanSensing) Predicate "%s" does not have either a function or processing information' % pred_name)
                        continue
                    python_string = "self.scripts." + pred_name + "(res, self.params[pred_name][self.clients_sub_idx[i]])"
                else:
                    rospy.logerr('KCL: (RosplanSensing) Predicate "%s" does not have either a function or processing information' % pred_name)
                    continue

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
                        if changed:
                            self.sensed_services[pred_name + ':' + params] = (val, changed, False)
                        self.srv_mutex.release()
                else:
                    params = reduce(lambda r, x: r + ':' + x, self.params[pred_name][self.clients_sub_idx[i]]) if self.params[pred_name][self.clients_sub_idx[i]] else ''
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
                    if changed:
                        self.sensed_services[pred_name + ':' + params] = (result, changed, False)
                    self.srv_mutex.release()

            r.sleep()
        return None  # Finish thread

    def update_kb(self):
        kus = KnowledgeUpdateServiceArrayRequest()
        # Get info from KB functions and propositions (to know type of variable)
        # Fill update
        self.dump_cache_mutex.acquire(True)
        self.mutex.acquire(True)
        sensed_predicates = self.sensed_topics.copy()
        self.mutex.release()
        self.srv_mutex.acquire(True)
        sensed_predicates.update(self.sensed_services.copy())
        self.srv_mutex.release()

        for predicate, (val, changed, updated) in sensed_predicates.items():
            if updated:
                continue

            if predicate in self.sensed_topics:
                self.mutex.acquire(True)
                self.sensed_topics[predicate] = (self.sensed_topics[predicate][0], False, True)  # Set to not changed and updated KB
                self.mutex.release()
            else:
                self.srv_mutex.acquire(True)
                self.sensed_services[predicate] = (self.sensed_services[predicate][0], False, True)  # Set to not changed and updated KB
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

            # TODO what to do if no parameters specified? iterate over all the instantiated parameters and add them?
            for i, param in enumerate(predicate_info):
                kv = KeyValue()
                kv.key = 'p' + str(i)
                kv.value = param
                ki.values.append(kv)


            kus.update_type += np.array(kus.ADD_KNOWLEDGE).tostring()
            kus.knowledge.append(ki)
        self.dump_cache_mutex.release()

        # Update the KB with the full array
        if len(kus.update_type) > 0:
            try:
                self.update_kb_srv.call(kus)
            except Exception as e:
                rospy.logerr("KCL (SensingInterface) Failed to update knowledge base: %s" % e.message)

    def get_kb_attribute(self, attribute_name):
        try:
            request = GetAttributeServiceRequest(attribute_name)
            ret = self.get_state_propositions_srv.call(request)
            if len(ret.attributes) > 0:
                return ret.attributes
            return self.get_state_functions_srv.call(request).attributes
        except Exception as e:
            rospy.logwarn("KCL (SensingInterface) Failed to call knowledge base for details: %s" % e.message)
            return []

    def kb_update_status(self, msg):

        if not msg.last_update_client == rospy.get_name():  # if I did not update the KB....

            self.dump_cache_mutex.acquire(True)
            self.mutex.acquire(True)
            self.srv_mutex.acquire(True)

            # Dump cache, it will be reloaded in the next update
            for predicate, (val, changed, updated) in self.sensed_topics.items():
                self.sensed_topics[predicate] = (val, changed, False)
            self.sensed_services.clear()

            self.srv_mutex.release()
            self.mutex.release()
            self.dump_cache_mutex.release()


if __name__ == "__main__":
    rospy.init_node('rosplan_sensing_interface', anonymous=False)
    rps = RosplanSensing()
    t = threading.Thread(target=RosplanSensing.call_services, args=(rps,))

    # main loop
    hz_rate = rospy.get_param('~main_rate', 10)
    rate = rospy.Rate(hz_rate)  # 10hz
    t.start()
    while not rospy.is_shutdown():
        # Note: spin is not needed in python as the callback methods are run in a different thread
        rps.update_kb()
        rate.sleep()
    t.join()
