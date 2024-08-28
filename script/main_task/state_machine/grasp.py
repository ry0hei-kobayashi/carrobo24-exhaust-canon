#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import smach
import numpy as np
from hsrlib.hsrif import HSRInterfaces
from tam_object_detection.srv import ObjectDetectionService, ObjectDetectionServiceRequest
from tamlib.tf import Transform
from tamlib.utils import Logger
from navigation_tools.nav_tool_lib import NavModule
from geometry_msgs.msg import Pose2D
from tam_grasp.srv import GraspPoseEstimationService, GraspPoseEstimationServiceRequest
from geometry_msgs.msg import Pose, Point, Quaternion


class GraspFromFloor(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes,
                            input_keys=['grasp_counter', 'search_locations', 'deposit_locations', 'detected_obj', 'depth'],
                            output_keys=['grasp_counter', 'detected_obj'])
        Logger.__init__(self)

        self.tamtf = Transform()
        self.hsrif = HSRInterfaces()
        self.nav_module = NavModule("pumas")
        self.srv_grasp = rospy.ServiceProxy("grasp_pose_estimation/service", GraspPoseEstimationService)
        rospy.wait_for_service("grasp_pose_estimation/service", timeout=100)

 
    '''
    def grasp_check(self, hand_motor_joint_before:float) -> bool:
        states = self.rosif.sub.joint_states()
        hand_motor_joint_after = round(states.position[20], 3)
        rospy.sleep(0.1)
        self.logdebug(f"hand_motor: {hand_motor_joint_after}")
        self.loginfo(f"差分：{abs(hand_motor_joint_after - hand_motor_joint_before)}")

        if abs(hand_motor_joint_after - hand_motor_joint_before) > 0.07: # TODO: ハイパーパラメータを定数に入れてまとめる
            return True
        else:
            return False
    '''

        

    def execute(self, userdata):
        self.hsrif.gripper.command(1.2)
        
        i = 0
        try:
            gpe_req = GraspPoseEstimationServiceRequest(
                depth=userdata.depth,
                mask=userdata.detected_obj[i]['seg'],
                pose=userdata.detected_obj[i]['pose'],
                camera_frame_id="head_rgbd_sensor_rgb_frame",
                frame_id="base_link",
            )
            #detections = self.
        except Exception as e:
            rospy.logerr(e)
            userdata.grasp_counter = 0
            return "nothing"

        gpe_res = self.srv_grasp(gpe_req)
        rospy.loginfo(userdata.detected_obj[i]['bbox'].name)
        rospy.loginfo(gpe_res.grasp.pose)
        
        pose = Pose()
        pose.position = Point(gpe_res.grasp.pose.position.x,
                              gpe_res.grasp.pose.position.y, 
                              gpe_res.grasp.pose.position.z)

        pose.orientation = Quaternion(gpe_res.grasp.pose.orientation.x, 
                                      gpe_res.grasp.pose.orientation.y, 
                                      gpe_res.grasp.pose.orientation.z, 
                                      gpe_res.grasp.pose.orientation.w)

        # move end effector

        self.hsrif.whole_body.move_end_effector_pose(pose, "task")

        #axis = (0, 0, 1)
        #self.hsrif.whole_body.move_end_effector_by_line(axis, 0.01, sync=True)
        print("3,grip")
        
        self.hsrif.gripper.apply_force(1.0)
        
        
        
        axis = (0, 0, -1)
        try:
            self.hsrif.whole_body.move_end_effector_by_line(axis, 0.8, sync=False)
        except Exception as e:
            goal = Pose2D(0.0, -0.2, 0.0)
            self.nav_module(pose, nav_type='hsr', nav_mode='rel', nav_timeout=0)
            rospy.logerr("持ち上げエラー")
        print("4,motiage")
        
        return "next"
