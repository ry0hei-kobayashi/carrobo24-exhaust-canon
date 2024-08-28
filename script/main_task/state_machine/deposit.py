#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
import rospy
import smach
from geometry_msgs.msg import Pose2D
from hsrlib.hsrif import HSRInterfaces
from hsrlib.rosif import ROSInterfaces
from tamlib.utils import Logger

from navigation_tools.nav_tool_lib import NavModule
from geometry_msgs.msg import Pose2D

class DepositObject(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes,
                            input_keys=['grasp_counter', 'position', 'detected_obj', 'deposit_locations', 'food_select'],
                            output_keys=['grasp_counter', 'position', 'detected_obj', 'food_select'])
        Logger.__init__(self)

        self.hsrif = HSRInterfaces()
        self.rosif = ROSInterfaces()
        self.nav_module = NavModule("pumas")
        self.category_lsit = {
            "food": [
                "master_chef_can",
                "cracker_box",
                "cheez_it_cracker_box",
                "domino_sugar_box",
                "jell_o_chocolate_pudding_box",
                "jell_o_strawberry_gelatin_box",
                "spam_potted_meat_can",
                "master_chef_coffee_can",
                "starkist_tuna_fish_can",
                "pringles_chips_can",
                "frenchs_mustard_bottle",
                "tomato_soup_can",
                "plastic_banana",
                "plastic_strawberry",
                "plastic_apple",
                "plastic_lemon",
                "plastic_peach",
                "plastic_pear",
                "plastic_orange",
                "plastic_plum",
                "tomato_soup_can",
                "mustard_bottle",
                "tuna_fish_can",
                "pudding_box",
                "gelatin_box",
                "potted_meat_can",
                "banana",
                "strawberry",
                "apple",
                "lemon",
                "peach",
                "pear",
                "orange",
                "plum",
                ],
            "kitchen": [
                "windex_spray_bottle",
                "scrub_cleanser_bottle",
                "scotch_brite_dobie_sponge",
                "pitcher_base",
                "pitcher_lid",
                "plate",
                "bowl",
                "spatula",
                "wine_glass",
                "mug",
                "bleach_cleanser",
                "windex_bottle",
                "sponge"],
            "tool": [
                "key",
                "bolt_and_nut",
                "clamp",
                "padlock"],      
            "shape": [
                "credit_card",
                "soccer_ball",
                "soft_ball",
                "baseball",
                "tennis_ball",
                "racquetball",
                "golf_ball",
                "marbles",
                "cups",
                "foam_brick",
                "dice",
                "rope",
                "chain",
                "mini_soccer_ball"],  
            "task": [
                "rubiks_cube",
                "colored_wood_blocks",
                "nine_peg_hole_test",
                "toy_airplane",
                "lego_duplo",
                "magazine",
                "t_shirt",
                "timer",
                "toy_airplane_tool",
                "toy_airplane_parts"],
            "orientation": [
                "large_marker",
                "spoon",
                "fork",
                "small_marker"]
        }

    def execute(self, userdata):
        i=0
        get_object=userdata.detected_obj[i]['bbox'].name
 
        if get_object in self.category_lsit["food"] and userdata.food_select%2==0:            
            category = "food1"
        elif get_object in self.category_lsit["food"] and userdata.food_select%2==1:            
            category = "food2"
        elif get_object in self.category_lsit["kitchen"]:            
            category = "kitchen"
        elif get_object in self.category_lsit["tool"]:            
            category = "tool"
        elif get_object in self.category_lsit["shape"]:            
            category = "shape"
        elif get_object in self.category_lsit["task"]:            
            category = "task"
        elif get_object in self.category_lsit["orientation"]:            
            category = "orientation"
        else:
            category = "unknown"

        userdata.food_select+=1

        # goal pose
        self.hsrif.whole_body.move_to_joint_positions(
            {
            'arm_lift_joint': 0.2,
            'arm_flex_joint': -1.0,
            'arm_roll_joint': 0.0,
            'wrist_flex_joint': -0.65,
            'wrist_roll_joint': 0.0,
            'head_pan_joint': 0.0,
            'head_tilt_joint': 0.0
            }, 
            sync=True
        )

        x=userdata.deposit_locations[category][0]
        y=userdata.deposit_locations[category][1]
        yaw=userdata.deposit_locations[category][2]

        # navigation
        goal = Pose2D(x, y, yaw)
        #self.nav_module.nav_goal(goal, nav_type="pumas", nav_mode="abs", nav_timeout=0, goal_distance=0) # main branch
        self.nav_module.nav_goal(goal, nav_type="pumas", nav_mode="abs", nav_timeout=0, goal_distance=0,
                                 angle_correction=True, obstacle_detection=False) # motion_synth

        self.hsrif.gripper.command(1.2)

        current_joints = self.rosif.sub.get_arm_joint_positions(latest=True)
        self.rosif.pub.arm_command(
            {"arm_flex_joint": current_joints["arm_flex_joint"] + np.deg2rad(45.0)},
            time=0.1,
        )
        self.rosif.pub.command_velocity_in_sec(-0.3, 0, 0, 1)

        if len(userdata.detected_obj)==0:
            userdata.grasp_counter = 0
            return "re_recog"

        if userdata.grasp_counter < 3:
            userdata.grasp_counter += 1
            userdata.detected_obj.pop(0)
            rospy.loginfo(len(userdata.detected_obj))
            return "next"
        else:
            userdata.grasp_counter = 0
            return "re_recog"
