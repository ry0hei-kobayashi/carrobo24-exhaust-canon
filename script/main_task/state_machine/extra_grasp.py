#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import smach
import numpy as np
from hsrlib.hsrif import HSRInterfaces
# from hsrnavlib import LibHSRNavigation
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
                            input_keys=['object', 'place', 'places'],
                            output_keys=['object', 'place', 'places'])
        Logger.__init__(self)

        self.tamtf = Transform()
        self.hsrif = HSRInterfaces()
        # self.navigation = LibHSRNavigation()
        self.nav_module = NavModule("pumas")
        
        # srv_detection = rospy.ServiceProxy("hsr_head_rgbd/object_detection/service", ObjectDetectionService)
        # rospy.wait_for_service("hsr_head_rgbd/object_detection/service", timeout=100)
        self.srv_grasp = rospy.ServiceProxy("grasp_pose_estimation/service", GraspPoseEstimationService)
        rospy.wait_for_service("grasp_pose_estimation/service", timeout=100)
        
        # Service
        self.srv_detection = rospy.ServiceProxy(
            "hsr_head_rgbd/object_detection/service", ObjectDetectionService
        )
        self.srv_detection(ObjectDetectionServiceRequest())
        
    # def grasp_failure(self):
    #     self.logwarn("Grasp FAILURE")
    #     userdata.grasp_counter += 1
    #     try:
    #         # 失敗したらオブジェクトリストから先頭を消す
    #         userdata.detected_obj.pop(0)
    #     except Exception as e:
    #         rospy.loginfo('オブジェクトもうない')
    #     if userdata.grasp_counter > 3:
    #         # ３回失敗して認識へ
    #         userdata.grasp_counter = 0
    #         return "nothing"
    #     return "failure"

    def execute(self, userdata):
        
        self.hsrif.gripper.command(1.2)
        
        self.hsrif.whole_body.move_to_joint_positions(
            {
                "arm_lift_joint": 0.0,
                "arm_flex_joint": np.deg2rad(0.0),
                "arm_roll_joint": np.deg2rad(90.0),
                "wrist_roll_joint": np.deg2rad(0.0),
                "wrist_flex_joint": np.deg2rad(-110.0),
                "head_pan_joint": np.deg2rad(60),
                "head_tilt_joint": np.deg2rad(-40),
            }, 
            sync=True
        )

        # object detection
        det_req = ObjectDetectionServiceRequest(use_latest_image=True)
        detections = self.srv_detection(det_req).detections

        for idx in range(len(detections.bbox)):
            if userdata.object==detections.bbox[idx].name:
                break
        
        try:
            gpe_req = GraspPoseEstimationServiceRequest(
                depth=detections.depth,
                mask=detections.segments[idx],
                pose=detections.pose[idx],
                camera_frame_id="head_rgbd_sensor_rgb_frame",
                frame_id="base_link",
            )
        except Exception as e:
            rospy.logerr(e)
            return "loop"
        # print(gpe_req)
        gpe_res = self.srv_grasp(gpe_req)
        
        pose = Pose()
        pose.position = Point(gpe_res.grasp.pose.position.x,    gpe_res.grasp.pose.position.y, gpe_res.grasp.pose.position.z)
        pose.orientation = Quaternion(gpe_res.grasp.pose.orientation.x, gpe_res.grasp.pose.orientation.y, gpe_res.grasp.pose.orientation.z, gpe_res.grasp.pose.orientation.w)
        # move end effector
        self.hsrif.whole_body.move_end_effector_pose(pose, "task")

        axis = (0, 0, 1)
        self.hsrif.whole_body.move_end_effector_by_line(axis, 0.01, sync=True)
        print("3,grip")
        
        self.hsrif.gripper.apply_force(1.0)
        
        
        #if userdata.obj_name in ["030_FORK", "031_SPOON","041_SMALL_MARKER", "026_SPONGE","024_BOWL","040_LARGE_MARKER"]:
        # m hand_motor_joint
        hand_joint = self.hsrif.whole_body.joint_positions['hand_motor_joint']
        rospy.loginfo("------------------------------------")
        rospy.loginfo('hand_motor_joint:={}'.format(hand_joint))
        rospy.loginfo("------------------------------------")
        #error_grasping = False
        #if (hand_joint > GRASP_THRESHOLD):
        #    return 'success'
        """else:
            error_grasping = True
            #userdata.objectstf = []
            omni_base.go_rel(-0.1, 0, 0)
            return 'drop'
        """
        
        axis = (0, 0, -1)
        try:
            self.hsrif.whole_body.move_end_effector_by_line(axis, 0.8, sync=False)
        except Exception as e:
            goal = Pose2D(0.0, -0.5, 0.0)
            self.nav_module(pose, nav_type='hsr', nav_mode='rel', nav_timeout=0)
            rospy.logerr("持ち上げエラー")
        print("4,motiage")
        
        return "next"
