#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
import rospy
import smach
from geometry_msgs.msg import Pose2D
from hsrlib.hsrif import HSRInterfaces
from hsrlib.rosif import ROSInterfaces
# from hsrnavlib import LibHSRNavigation
from tamlib.utils import Logger

from navigation_tools.nav_tool_lib import NavModule
from geometry_msgs.msg import Pose2D

class DepositObject(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes,
                            input_keys=['grasp_counter', 'position', 'detected_obj', 'deposit_locations'], output_keys=['grasp_counter', 'position', 'detected_obj'])
        Logger.__init__(self)

        self.hsrif = HSRInterfaces()
        self.rosif = ROSInterfaces()
        #self.navigation = LibHSRNavigation()
        self.nav_module = NavModule("pumas")

    def execute(self, userdata):
        category = "task"

        self.loginfo(category)

        # goal pose
        self.hsrif.whole_body.move_to_joint_positions(
            {
                "arm_lift_joint": 0.35,
                "arm_flex_joint": np.deg2rad(-90.0),
                "arm_roll_joint": np.deg2rad(0.0),
                "wrist_flex_joint": np.deg2rad(-90.0),
                "wrist_roll_joint": np.deg2rad(0.0),
                "head_pan_joint": 0.0,
                "head_tilt_joint": 0.0,
            }, 
            sync=True
        )

        # navigation
        goal = Pose2D(1.4, 0.0, np.deg2rad(-90.0))
        self.nav_module.nav_goal(goal, nav_type="pumas", nav_mode="abs", nav_timeout=0, goal_distance=0) # full definition

        self.hsrif.gripper.command(1.2)

        current_joints = self.rosif.sub.get_arm_joint_positions(latest=True)
        self.rosif.pub.arm_command(
            {"arm_flex_joint": current_joints["arm_flex_joint"] + np.deg2rad(45.0)},
            time=0.1,
        )
        self.rosif.pub.command_velocity_in_sec(-0.3, 0, 0, 1)

        if userdata.grasp_counter < 3:
            userdata.grasp_counter += 1
            return "next"
        else:
            userdata.grasp_counter = 0:
            return "re_recog"
