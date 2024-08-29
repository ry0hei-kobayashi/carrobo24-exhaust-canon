#/usr/bin/env python3
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
                            input_keys=['search_locations', 'deposit_locations', 'position', 'grasp_counter', 'detected_obj'], output_keys=['position', 'grasp_counter', 'detected_obj'])
        Logger.__init__(self)

        # self.hsrif = HSRInterfaces()
        
        #call class at only first time
        self.nav_module = NavModule("pumas")

    def execute(self, userdata):

        #x,y,yaw = 2.74,-0.17,-1.57
        if userdata.position == 5:
            userdata.position = 0
        
        x = userdata.search_locations[userdata.position][0]
        y = userdata.search_locations[userdata.position][1]
        yaw = userdata.search_locations[userdata.position][2] #誤差radで与える
        
        # set to the goal point
        goal = Pose2D(x, y, yaw)
        
        #call nav function 
        #self.nav_module.nav_goal(goal, nav_type="pumas", nav_mode="abs", nav_timeout=1, goal_distance=0) # main branch
        self.nav_module.nav_goal(goal, nav_type="pumas", nav_mode="abs", nav_timeout=0, goal_distance=0, 
                                 angle_correction=False, obstacle_detection=False) # motion_synth branch 

        if len(userdata.detected_obj)==0:
            # 見つけられなかったら認識に戻る
            userdata.grasp_counter = 0
            return "recog"

        if userdata.grasp_counter < 3:
            rospy.loginfo(len(userdata.detected_obj))
            return "grasp"
        else:
            # サーチポジション次の場所へ
            userdata.position += 1
            if userdata.position > 5:
                userdata.position = 0
            userdata.grasp_counter = 0
            return "recog"

if __name__ == "__main__":
    rospy.init_node('state_machine_nav')

    sm = smach.StateMachine(outcomes='exit')
    with sm:
        smach.StateMachine.add('DEBUG', GoToFloor(),
                               transitions={'next': 'DEBUG'})
    sm.execute()

