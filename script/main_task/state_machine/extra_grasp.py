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

import tf.transformations as tf

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

        # x = userdata.places[userdata.place][0]
        # y = userdata.places[userdata.place][1]
        # yaw = userdata.places[userdata.place][2] #誤差radで与える
        
        # # set to the goal point
        # goal = Pose2D(x, y, yaw)
        
        # #call nav function 
        # #self.nav_module.nav_goal(goal, nav_type="pumas", nav_mode="abs", nav_timeout=1, goal_distance=0) # main branch
        # self.nav_module.nav_goal(goal, nav_type="pumas", nav_mode="abs", nav_timeout=0, goal_distance=0, 
        #                          angle_correction=True, obstacle_detection=True) # motion_synth branch 

        self.hsrif.gripper.command(1.2)
        
        self.hsrif.whole_body.move_to_joint_positions(
            {
                "arm_lift_joint": 0.0,
                "arm_flex_joint": np.deg2rad(0.0),
                "arm_roll_joint": np.deg2rad(90.0),
                "wrist_roll_joint": np.deg2rad(0.0),
                "wrist_flex_joint": np.deg2rad(-110.0),
                "head_pan_joint": np.deg2rad(0),
                "head_tilt_joint": np.deg2rad(-30),
            }, 
            sync=True
        )

        # object detection
        det_req = ObjectDetectionServiceRequest(use_latest_image=True)
        detections = self.srv_detection(det_req).detections

        for idx, obj in enumerate(detections.bbox):
            if userdata.object in obj.name:
                rospy.logerr(f"find{userdata.object}")
                break
            else:
                if len(detections.bbox)==idx+1:
                    userdata.place="table"
                    return "search"
                else:
                    pass
        
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
        
        self.hsrif.whole_body.move_to_joint_positions(
            {
                "arm_lift_joint":  0.2,
                "arm_flex_joint":  -1.57,
                "arm_roll_joint":  -1.57,
                "wrist_roll_joint": 0.0,
                "wrist_flex_joint": 0.0,
            }, 
            sync=True
        )
        pose = Pose()
        pose.position = Point(gpe_res.grasp.pose.position.x,    gpe_res.grasp.pose.position.y, gpe_res.grasp.pose.position.z-0.11)
        # pose.orientation = Quaternion(gpe_res.grasp.pose.orientation.x, gpe_res.grasp.pose.orientation.y, gpe_res.grasp.pose.orientation.z, gpe_res.grasp.pose.orientation.w)
        # # Z軸周りに90度回転
        # rotation_quaternion = tf.quaternion_from_euler(0, 0, tf.pi/2)

        # # 結果としてのクォータニオンを取得
        # x, y, z, w = rotation_quaternion
        pose.orientation = Quaternion(0.0, 0.7071, 0.0, 0.7071)
        # move end effector
        self.hsrif.whole_body.move_end_effector_pose(pose, "task")

        # rospy.loginfo("what")
        # axis = (0, 1, 0)
        # self.hsrif.whole_body.move_end_effector_by_line(axis, 0.01, sync=True)
        # print("3,grip")
        
        self.hsrif.gripper.apply_force(1.0)
        
        
        # #if userdata.obj_name in ["030_FORK", "031_SPOON","041_SMALL_MARKER", "026_SPONGE","024_BOWL","040_LARGE_MARKER"]:
        # # m hand_motor_joint
        # hand_joint = self.hsrif.whole_body.joint_positions['hand_motor_joint']
        # rospy.loginfo("------------------------------------")
        # rospy.loginfo('hand_motor_joint:={}'.format(hand_joint))
        # rospy.loginfo("------------------------------------")
        # #error_grasping = False
        # #if (hand_joint > GRASP_THRESHOLD):
        # #    return 'success'
        # """else:
        #     error_grasping = True
        #     #userdata.objectstf = []
        #     omni_base.go_rel(-0.1, 0, 0)
        #     return 'drop'
        # """
        
        # axis = (0, 0, -1)
        # try:
        #     self.hsrif.whole_body.move_end_effector_by_line(axis, 0.8, sync=False)
        # except Exception as e:
        #     # goal = Pose2D(0.0, -0.5, 0.0)
        #     # self.nav_module(pose, nav_type='hsr', nav_mode='rel', nav_timeout=0)
        #     rospy.logerr("持ち上げエラー")
        # print("4,motiage")

        userdata.place="person"
        
        return "next"

if __name__ == "__main__":
    rospy.init_node('state_machine_nav')

    sm = smach.StateMachine(outcomes='exit')
    sm.userdata.object="orange"
    sm.userdata.place="shelf"
    sm.userdata.places = {
        #key:{x,y,yaw}
        "table": (0.74, 0.9, 1.57  ),
        "shelf": (2.11, 3.9, 1.57  ),
        "person": (0.0918, 2.9, 0  ),
        }
    with sm:
        smach.StateMachine.add('DEBUG', GraspFromFloor(),
                               transitions={'next': 'DEBUG'})
    sm.execute()