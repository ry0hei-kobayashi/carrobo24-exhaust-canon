#/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import smach
import numpy as np
from hsrlib.hsrif import HSRInterfaces
from hsrlib.utils import description, utils
from tam_object_detection.srv import ObjectDetectionService, ObjectDetectionServiceRequest
from tamlib.tf import Transform
from tamlib.utils import Logger
from navigation_tools.nav_tool_lib import NavModule
from geometry_msgs.msg import Pose2D, WrenchStamped
from tam_grasp.srv import GraspPoseEstimationService, GraspPoseEstimationServiceRequest
from geometry_msgs.msg import Pose, Point, Quaternion
from hsrb_interface.exceptions import *
from . import action_robot

GRASP_THRESHOLD = -0.85


class GraspFromFloor(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes,
                            input_keys=['grasp_counter', 'search_locations', 'deposit_locations', 'detected_obj', 'depth'],
                            output_keys=['grasp_counter', 'detected_obj'])
        Logger.__init__(self)

        self.tamtf = Transform()
        self.hsrif = HSRInterfaces()
        self.description = description.load_robot_description()
        self.nav_module = NavModule("pumas")
        self.srv_grasp = rospy.ServiceProxy("grasp_pose_estimation/service", GraspPoseEstimationService)

        #self.a_robot = action_robot.RobotWithAction()
        rospy.wait_for_service("grasp_pose_estimation/service", timeout=100)

        #wrench
        self.a_robot = action_robot.RobotWithAction()
        self.threshold = -28.0
        self.current_val = None

 
    def grasp_check(self, hand_motor_joint_before:float) -> bool:
        states = self.rosif.sub.joint_states()
        hand_motor_joint_after = round(states.position[20], 3)
        rospy.sleep(0.1)
        self.loginfo(f"hand_motor: {hand_motor_joint_after}")
        self.logerr("grasp.->chk grasping")
        #self.loginfo(f"差分：{abs(hand_motor_joint_after - hand_motor_joint_before)}

        if abs(hand_motor_joint_after - hand_motor_joint_before) > 0.07: 
            return True
        else:
            return False

    def wrench_cb(self, msg):
        try:
            self.current_value = msg.wrench.force.z  # TODO
            if self.current_value > self.threshold:
                self.a_robot.cancel_arm()
                self.pushed = True
                return
        except:
            rospy.logerr("for traceback")
            import traceback
            traceback.print_exc()
    
    def wrench_gripper_cb(self, msg):
        try:
            self.current_value = msg.wrench.force.z  # TODO
            if self.current_value > self.threshold_for_gripper:
                self.pushed_for_gripper = True
                self.a_robot.applyforce_client.cancel_goal()
        except:
            rospy.logerr("for traceback")
            import traceback
            traceback.print_exc()
    
        

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
        except Exception as e:
            rospy.logerr(e)
            userdata.grasp_counter = 0
            return "nothing"

        gpe_res = self.srv_grasp(gpe_req)
        if gpe_res.success is False:
            return"nothing"
        rospy.loginfo(userdata.detected_obj[i]['bbox'].name)
        rospy.loginfo(gpe_res.grasp.pose)

        set_grasp_pose = self.tamtf.get_pose_with_offset(
            self.description.frame.base,
            'task',
            gpe_res.grasp.pose,
            'grasp'
        )


        self.hsrif.whole_body.linear_weight = 1
        self.hsrif.whole_body.angular_weight = 100
        #self.hsrif.whole_body.joint_weights = {
        #    "arm_lift_joint": 1.0,
        #    "arm_flex_joint": 100.0,
        #    "arm_roll_joint": 1.0,
        #    "wrist_flex_joint": 1.0,
        #    "wrist_roll_joint": 1.0,
        #}

        
        pose = Pose()
        pose.position = Point(gpe_res.grasp.pose.position.x,
                              gpe_res.grasp.pose.position.y, 
                              gpe_res.grasp.pose.position.z - 0.05)

        pose.orientation = Quaternion(gpe_res.grasp.pose.orientation.x, 
                                      gpe_res.grasp.pose.orientation.y, 
                                      gpe_res.grasp.pose.orientation.z, 
                                      gpe_res.grasp.pose.orientation.w)

        rospy.logwarn('grasp.-> contact to the object')
        self.hsrif.whole_body.move_end_effector_pose(pose, "task")
        # move end effector
        
        #pose = Pose()
        #pose.position = Point(gpe_res.grasp.pose.position.x,
        #                      gpe_res.grasp.pose.position.y, 
        #                      gpe_res.grasp.pose.position.z - 0.0)

        #pose.orientation = Quaternion(gpe_res.grasp.pose.orientation.x, 
        #                              gpe_res.grasp.pose.orientation.y, 
        #                              gpe_res.grasp.pose.orientation.z, 
        #rospy.logwarn('grasp.-> contact to the object')
        #self.hsrif.whole_body.move_end_effector_pose(pose, "task")
        #                              gpe_res.grasp.pose.orientation.w)

        #z_offset = 0.05
        #try:
        #    rospy.logwarn('grasp.-> contact to the object')
        #    self.hsrif.whole_body.move_end_effector_pose(pose, "task")
        #    try:
        #        with TemporarySubscriber('/hsrb/wrist_wrench/raw', WrenchStamped, self.wrench_cb):
        #            rate = rospy.Rate(10)
        #            rospy.logwarn('grasp.-> arm down down down')
        #            pose.position.z +=  0.01
        #            self.hsrif.whole_body.move_end_effector_pose((pose), "task")

        #    except FollowTrajectoryError: 
        #        rospy.logwarn('grasp.-> arm reached to the object or floor')
        #        pose.position.z -= 0.04
        #        self.hsrif.whole_body.move_end_effector_pose(pose, "task")

        #    except:
        #        pose.position.z -= 0.03
        #        self.hsrif.whole_body.move_end_effector_pose(pose, "task")

        #except:
        #    self.hsrif.whole_body.move_end_effector_pose(geometry.pose(z=0.03), "task")

        ##gripper force
        self.pushed_for_gripper = False
        with TemporarySubscriber('/hsrb/wrist_wrench/raw', WrenchStamped, self.wrench_cb):
            rate = rospy.Rate(100)
            rospy.logwarn('grasp.-> gripper contact')
            gripper_val = 0
            while not rospy.is_shutdown() and not self.pushed_for_gripper:
                hand_motor_joint = self.hsrif.whole_body.joint_positions['hand_motor_joint']
                self.a_robot.gripper_applyforce(1.0)
                if hand_motor_joint < 1:
                    break
                rate.sleep()
            self.a_robot.apply_force_client.cancel_goal()


        #axis = (0, 0, 1)
        #self.hsrif.whole_body.move_end_effector_by_line(axis, 0.01, sync=True)
        
        self.hsrif.gripper.apply_force(1.0)
        
        axis = (0, 0, -1)
        try:
            self.hsrif.whole_body.move_end_effector_by_line(axis, 0.5, sync=False)
        except Exception as e:
            goal = Pose2D(0.15, 0.0, 0.0)
            self.nav_module(pose, nav_type='hsr', nav_mode='rel', nav_timeout=0)
            self.hsrif.whole_body.move_end_effector_by_line(axis, 0.45, sync=False)
            rospy.logerr("持ち上げエラー")


        # m hand_motor_joint
        hand_joint = self.hsrif.whole_body.joint_positions['hand_motor_joint']
        rospy.loginfo("------------------------------------")
        rospy.loginfo('hand_motor_joint:={}'.format(hand_joint))
        rospy.loginfo("------------------------------------")
 
        if (hand_joint > GRASP_THRESHOLD):
            return 'next'
        else:
            return 'failure' 
 
        

class TemporarySubscriber:
    def __init__(self, name, msg, cb, *args):
        self.name = name
        self.msg = msg
        self.cb = cb
        self.args = args

    def __enter__(self):
        self.sub = rospy.Subscriber(self.name, self.msg, self.cb, *self.args)
        return self.sub

    def __exit__(self, exctype, excval, traceback):
        self.sub.unregister()

