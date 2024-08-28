#!/usr/bin/env python3
# -*-encoding:utf-8-*-
import rospy
from hsrlib.hsrif import HSRInterfaces

def lec_arm_moveit():

    # Declaration node name
    rospy.init_node('whole_body_task', anonymous=True)
    
    # Declaration module
    hsrif = HSRInterfaces()

    # default pose
    #hsrif.whole_body.move_to_neutral(sync=True)

    # move joint function
    hsrif.whole_body.move_to_joint_positions(
        {
            'arm_lift_joint': 0.18,
            'arm_flex_joint': -1.5,
            'arm_roll_joint': 0.0,
            'wrist_flex_joint': -0.2,
            'wrist_roll_joint': 0.0,
            'head_pan_joint': 0.0,
            'head_tilt_joint': 0.0
        },
        sync=True)
    rospy.spin()


if __name__ == '__main__':
    lec_arm_moveit()
    
    
"""
tool
            'arm_lift_joint': 0.2,
            'arm_flex_joint': -1.0,
            'arm_roll_joint': 0.0,
            'wrist_flex_joint': -0.65,
            'wrist_roll_joint': 0.0,
            'head_pan_joint': 0.0,
            'head_tilt_joint': 0.0
            
shape
            'arm_lift_joint': 0.0,
            'arm_flex_joint': -1.0,
            'arm_roll_joint': 0.0,
            'wrist_flex_joint': -0.65,
            'wrist_roll_joint': 0.0,
            'head_pan_joint': 0.0,
            'head_tilt_joint': 0.0         
            
kitchen           
            'arm_lift_joint': 0.3,
            'arm_flex_joint': -1.5,
            'arm_roll_joint': 0.0,
            'wrist_flex_joint': -0.1,
            'wrist_roll_joint': 0.0,
            'head_pan_joint': 0.0,
            'head_tilt_joint': 0.0
            
orientation
            'arm_lift_joint': 0.28,
            'arm_flex_joint': -1.5,
            'arm_roll_joint': 0.0,
            'wrist_flex_joint': -0.2,
            'wrist_roll_joint': 0.0,
            'head_pan_joint': 0.0,
            'head_tilt_joint': 0.0        
            
food
            'arm_lift_joint': 0.24,
            'arm_flex_joint': -1.5,
            'arm_roll_joint': 0.0,
            'wrist_flex_joint': -0.2,
            'wrist_roll_joint': 0.0,
            'head_pan_joint': 0.0,
            'head_tilt_joint': 0.0            
            
task,unknow
            'arm_lift_joint': 0.18,
            'arm_flex_joint': -1.5,
            'arm_roll_joint': 0.0,
            'wrist_flex_joint': -0.2,
            'wrist_roll_joint': 0.0,
            'head_pan_joint': 0.0,
            'head_tilt_joint': 0.0           
            
            
            
            
            
                      
"""
