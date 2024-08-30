#!/usr/bin/env python

import sys

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus, GoalID
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import ListControllers
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tmc_control_msgs.msg import GripperApplyEffortAction
from tmc_control_msgs.msg import GripperApplyEffortGoal

import tf.transformations

import hsrb_interface.exceptions as exceptions
from hsrb_interface import geometry

_GRIPPER_FOLLOW_TRAJECTORY_TIMEOUT = 20.0
_GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD = 0.8
_GRIPPER_GRASP_TIMEOUT = 20.0
_HAND_MOMENT_ARM_LENGTH = 0.07

class RobotWithAction:
    def __init__(self):
        self.initialize_end_effector()
        self.initialize_head()
        self.initialize_arm()
        self.initialize_base()
        
    def initialize_head(self):
        self.head_cli = actionlib.SimpleActionClient(
            '/hsrb/head_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("Connecting HEAD controller manager")
        self.head_cli.wait_for_server()

        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers', ListControllers)
        running = False
        
        while running is False:
            rospy.sleep(0.1)
            for c in list_controllers().controller:
                if c.name == 'head_trajectory_controller' and c.state == 'running':
                    running = True
        rospy.loginfo("Compleated to Connected HEAD controller manager")

    def move_head(self, pan, tilt, wait_result=False):
        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = JointTrajectoryPoint()
        p.positions = [pan, tilt]
        p.velocities = [0, 0]
        p.time_from_start = rospy.Time(1)
        traj.points = [p]
        goal.trajectory = traj
        self.head_cli.send_goal(goal)

        if wait_result:
            self.head_cli.wait_for_result()

    def initialize_arm(self):
        self.cancel_arm_pub = rospy.Publisher("/hsrb/arm_trajectory_controller/follow_joint_trajectory/cancel", GoalID, queue_size=1)
        self.arm_cli = actionlib.SimpleActionClient(
            '/hsrb/arm_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("Connecting ARM controller manager")
        self.arm_cli.wait_for_server()
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy(
            '/hsrb/controller_manager/list_controllers', ListControllers)
        running = False
        while running is False:
            rospy.sleep(0.1)
            for c in list_controllers().controller:
                if c.name == 'arm_trajectory_controller' and c.state == 'running':
                    running = True
        rospy.loginfo("Compleated to Connected ARM controller manager")

    def move_arm(self, arm_lift_joint, arm_flex_joint, arm_roll_joint,
                 wrist_flex_joint, wrist_roll_joint, wait_result=False):
        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                            "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        p = JointTrajectoryPoint()
        p.positions = [arm_lift_joint, arm_flex_joint, arm_roll_joint,
                       wrist_flex_joint, wrist_roll_joint]
        p.velocities = [0, 0, 0, 0, 0]
        p.time_from_start = rospy.Time(1)
        traj.points = [p]
        goal.trajectory = traj

        self.arm_cli.send_goal(goal)
        if wait_result:
            self.arm_cli.wait_for_result()

    def move_to_go(self, wait_result=False):
        self.move_head(0,0)
        self.move_arm(0.0, 0.0, -1.57, -1.57, 0.0, wait_result)

    def move_to_neutral(self, wait_result=False):
        self.move_head(0,0)
        self.move_arm(0.0, 0.0, 0.0, -1.57, 0.0, wait_result)

    def cancel_arm(self):
        msg = GoalID()
        self.cancel_arm_pub.publish(msg)

    def initialize_base(self):
        rospy.loginfo("Connecting BASE controller manager")
        self.base_cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self.base_cli.wait_for_server()
        rospy.loginfo("Compleated to Connected BASE controller manager")
        
    def go_abs(self, x, y, yaw, wait_result=True):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(x, y, 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation = Quaternion(*quat)
        
        goal = MoveBaseGoal()
        goal.target_pose = pose
        
        self.base_cli.send_goal(goal)
        
        # wait for the action server to complete the order
        if wait_result:
            self.base_cli.wait_for_result()

    def initialize_end_effector(self):
        rospy.loginfo("Connecting END-EFFECTOR controller manager")

        self.follow_joint_trajectory_client = actionlib.SimpleActionClient("/hsrb/gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.follow_joint_trajectory_client.wait_for_server()

        self.grasp_client = actionlib.SimpleActionClient("/hsrb/gripper_controller/grasp",GripperApplyEffortAction)
        self.grasp_client.wait_for_server()

        self.apply_force_client = actionlib.SimpleActionClient("/hsrb/gripper_controller/apply_force",GripperApplyEffortAction)
        self.apply_force_client.wait_for_server()

        rospy.loginfo("Compleated to Connected END-EFFECTOR controller manager")

    def gripper_command(self, open_angle, motion_time=1.0):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["hand_motor_joint"]
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=[open_angle],
                                 time_from_start=rospy.Duration(motion_time))]

        self.follow_joint_trajectory_client.send_goal(goal)
        timeout = rospy.Duration(_GRIPPER_FOLLOW_TRAJECTORY_TIMEOUT)
        try:
            if self.follow_joint_trajectory_client.wait_for_result(timeout):
                s = self.follow_joint_trajectory_client.get_state()
                if s != actionlib.GoalStatus.SUCCEEDED:
                    msg = "Failed to follow commanded trajectory"
                    raise exceptions.GripperError(msg)
            else:
                self.follow_joint_trajectory_client.cancel_goal()
                raise exceptions.GripperError("Timed out")
        except KeyboardInterrupt:
            self.follow_joint_trajectory_client.cancel_goal()

    def gripper_applyforce(self, effort, delicate=False):
        if effort < 0.0:
            msg = "negative effort is set"
            raise exceptions.GripperError(msg)
        goal = GripperApplyEffortGoal()
        goal.effort = - effort * _HAND_MOMENT_ARM_LENGTH
        client = self.grasp_client
        if delicate:
            if effort < _GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD:
                goal.effort = effort
                client = self.apply_force_client
            else:
                rospy.logwarn("Since effort is high, force control become invalid.")

        client.send_goal(goal)
        # try:
        #     timeout = rospy.Duration(_GRIPPER_GRASP_TIMEOUT)
        #     if client.wait_for_result(timeout):
        #         client.get_result()
        #         state = client.get_state()
        #         if state != actionlib.GoalStatus.SUCCEEDED:
        #             raise exceptions.GripperError("Failed to apply force")
        #     else:
        #         client.cancel_goal()
        #         raise exceptions.GripperError("Timed out")
        # except KeyboardInterrupt:
        #     client.cancel_goal()
        
if __name__ == "__main__":
    import hsrb_interface
    robot = hsrb_interface.Robot()
    omni_base = robot.get("omni_base")
    whole_body = robot.get("whole_body")

    ra = RobotWithAction()
    #arm_lift_joint, arm_flex_joint, arm_roll_joint, wrist_flex_joint, wrist_roll_joint
    # ra.move_arm(0,0,0,0,0,True)
    #ra.move_arm(0.1, -0.99, 0.0, -1.04, 0.0, False)
    #ra.go_abs(0,0,0)
    #while not rospy.is_shutdown():
    # ra.move_to_neutral(wait_result=True)
    # omni_base.go_rel(0.5,0,0)
    # ra.move_to_go(wait_result=True)

    # ra.move_to_neutral(wait_result=False)
    # omni_base.go_rel(-0.5,0,0)


    #ra.gripper_command(1.0)
    ra.move_arm(0.17, 0, -1.57, -1.57,0,True)
    #ra.cancel_arm()

    #debug for open drawer
    #ra.move_arm(0.01, -1.84, 0.0, 0.32, -1.48, False)
    # omni_base.go_abs(0.97,-0.01,-1.57)
    # ra.gripper_applyforce(0.7)
    # whole_body.move_end_effector_pose(geometry.pose(z=-0.3), "hand_palm_link")
    # ra.gripper_command(1.0)
    
    # omni_base.go_rel(0,0,0.5)
    #ra.move_arm(0.22, -1.84, 0.0, 0.32, -1.48, False)
    # omni_base.go_abs(0.98, -0.12, -1.57)
    # ra.gripper_applyforce(0.7)
    # whole_body.move_end_effector_pose(geometry.pose(z=-0.3/2), "hand_palm_link")
    # ra.gripper_command(1.0)

    # omni_base.go_abs(1.32,-0.10,-1.57)
    # ra.move_arm(0.01, -1.84, 0.0, 0.32, -1.48, True)
    # omni_base.go_rel(0.05,0,0)
    # ra.gripper_applyforce(0.7)
    # whole_body.move_end_effector_pose(geometry.pose(z=-0.3), "hand_palm_link")
    # ra.gripper_command(1.0)

    # ra.move_head(-1.57,0)
