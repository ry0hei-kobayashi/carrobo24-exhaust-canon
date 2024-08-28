#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
import rospy
import smach
from geometry_msgs.msg import Pose2D
from hsrlib.hsrif import HSRInterfaces
from tamlib.utils import Logger

from navigation_tools.nav_tool_lib import NavModule
from geometry_msgs.msg import Pose2D


class GoToFloor(smach.State, Logger):
    def __init__(self, outcomes):
    #def __init__(self, outcomes=['next']):
        smach.State.__init__(self, outcomes,
                            input_keys=['search_locations', 'deposit_locations', 'position'], output_keys=['position'])
        Logger.__init__(self)

        # self.hsrif = HSRInterfaces()
        
        #call class at only first time
        self.nav_module = NavModule("pumas")

    def execute(self, userdata):

        #x,y,yaw = 2.74,-0.17,-1.57
        
        x = userdata.search_locations[userdata.position][0]
        y = userdata.search_locations[userdata.position][1]
        yaw = userdata.search_locations[userdata.position][2] #誤差radで与える
        
        # set to the goal point
        goal = Pose2D(x, y, yaw)
        
        #call nav function 
        #self.nav_module.nav_goal(goal, nav_type="pumas", nav_mode="abs", nav_timeout=1, goal_distance=0) # main branch
        self.nav_module.nav_goal(goal, nav_type="pumas", nav_mode="abs", nav_timeout=0, goal_distance=0, 
                                 angle_correction=True, obstacle_detection=False) # motion_synth branch 

        # サーチポジション次の場所へ
        userdata.position += 1
        if userdata.position > 5:
            userdata.position = 0

        return "next"

if __name__ == "__main__":
    rospy.init_node('state_machine_nav')

    sm = smach.StateMachine(outcomes='exit')
    with sm:
        smach.StateMachine.add('DEBUG', GoToFloor(),
                               transitions={'next': 'DEBUG'})
    sm.execute()

