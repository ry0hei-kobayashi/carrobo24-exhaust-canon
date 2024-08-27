#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
import rospy
import smach
from geometry_msgs.msg import Pose2D
from hsrlib.hsrif import HSRInterfaces
# from hsrnavlib import LibHSRNavigation
from tamlib.utils import Logger

from navigation_tools.nav_tool_lib import NavModule
from geometry_msgs.msg import Pose2D


class GoToFloor(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes,
                            input_keys=['search_locations', 'deposit_locations', 'position'])
        Logger.__init__(self)

        self.hsrif = HSRInterfaces()
        
        #call class at only first time
        self.nav_module = NavModule("pumas")

    def execute(self, userdata):
        #print(goto1)
        self.hsrif.whole_body.move_to_joint_positions(
            {
                "arm_lift_joint": 0.0,
                "arm_flex_joint": np.deg2rad(0.0),
                "arm_roll_joint": np.deg2rad(90.0),
                "wrist_roll_joint": np.deg2rad(0.0),
                "wrist_flex_joint": np.deg2rad(-110.0),
                "head_pan_joint": 0.0,
                "head_tilt_joint": np.deg2rad(-40),
            }, 
            sync=True
        )
        
        x = userdata.search_locations[userdata.position][0]
        y = userdata.search_locations[userdata.position][1]
        yaw = userdata.search_locations[userdata.position][2]
        
        # set to the goal point
        goal = Pose2D(x, y, np.deg2rad(yaw))
        
        #call nav function 
        self.nav_module.nav_goal(goal, nav_type="pumas", nav_mode="abs", nav_timeout=0, goal_distance=0) # full definition

        # サーチポジション次の場所へ
        userdata.position += 1
        if userdata.position > 5:
            userdata.position = 0

        return "next"
