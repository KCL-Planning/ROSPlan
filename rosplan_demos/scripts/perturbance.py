#!/usr/bin/env python
import rospkg
import rospy
import sys
import random

from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayResponse, KnowledgeUpdateServiceArrayRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import *

rospy.wait_for_service('/rosplan_knowledge_base/update_array')
try:
    update = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
    arr = KnowledgeUpdateServiceArrayRequest()
    update_type = []
    call = False

    for i in [0,1,2]:

        if random.randint(1,100) <= 50:

            ki_remove = KnowledgeItem()
            ki_remove.knowledge_type = 1
            ki_remove.attribute_name = "machine_off"
            kv = KeyValue()
            kv.key = "m"
            kv.value =  "machine"+str(i)
            ki_remove.values.append(kv)
            arr.knowledge.append(ki_remove)
            update_type.append(2)

            ki_remove = KnowledgeItem()
            ki_remove.knowledge_type = 1
            ki_remove.attribute_name = "machine_on"
            kv = KeyValue()
            kv.key = "m"
            kv.value =  "machine"+str(i)
            ki_remove.values.append(kv)
            arr.knowledge.append(ki_remove)
            update_type.append(0)

            call = True

    if call:
        arr.update_type = update_type
        print arr
        update(arr)

except rospy.ServiceException, e:
    print "Unexpected perturbances not called"
