#!/usr/bin/env python

import rospy
import roslib
import os
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose

def spawn_model_in_gazebo(model_name, model_file, x, y, z):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        with open(model_file, 'r') as f:
            sdf_content = f.read()

        initial_pose = Pose()
        initial_pose.position.x = x
        initial_pose.position.y = y
        initial_pose.position.z = z

        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox(model_name, sdf_content, "", initial_pose, "world")

        rospy.loginfo(f"Spawned {model_name} at ({x}, {y}, {z})")

    except Exception as e:
        rospy.logerr(f"Failed to spawn {model_name}: {e}")

if __name__ == '__main__':
    rospy.init_node('spawn_models_in_gazebo')

    # Paths to the model SDF files
    path = roslib.packages.get_pkg_dir('hma_hsr_sim_world') + "/obj/ycb/"
    print(path)

    models = [
        ("orange",   path + "017_orange/orange.sdf", -0.3, 0.2, 0.5),
        ("soccer",   path + "053_mini_soccer_ball/mini_soccer_ball.sdf", 2.70, -0.8, 0.5),
        ("baseball", path + "055_baseball/baseball.sdf", 2.70, -0.95, 0.5),
        ("softball", path + "054_softball/softball.sdf", 2.70, -1.15, 0.5),
    ]
    print(models)

    for model_name, model_file, x, y, z in models:
        spawn_model_in_gazebo(model_name, model_file, x, y, z)
    rospy.spin()
