#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import smach
from std_msgs.msg import Bool
from hsrlib.hsrif import HSRInterfaces
from hsrlib.utils import utils
from std_srvs.srv import SetBool, SetBoolRequest
from tamlib.utils import Logger


class Init(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self)

        self.hsrif = HSRInterfaces()
        
        self.pub_open_drawer = rospy.Publisher(
            "/prepare_task_node/run_enable", Bool, queue_size=1)
        
        self.pub_time_supervisor = rospy.Publisher(
            "/manage_task_time_node/run_enable", Bool, queue_size=1)

        self.loginfo("init")

    def execute(self, userdata):
        self.loginfo("init")

        self.hsrif.whole_body.move_to_neutral()
        self.hsrif.gripper.command(0)

        self.pub_open_drawer.publish(Bool(True))
        
        while True:
            ready = rospy.wait_for_message('/manage_task_time_node/ready', Bool) 
            if ready.data:
                break
        
        input("Start >>> ")
        self.pub_time_supervisor.publish(Bool(True))
        return "next"


class Finish(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self)

    def execute(self, userdata):
        self.loginfo("Finished")
        return "finish"
