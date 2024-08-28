#!/usr/bin/env python

import rospy
import os
from gazebo_ros import spawn_model
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

        spawn_model.spawn_sdf_model(
            model_name,
            sdf_content,
            '',
            initial_pose,
            'world'
        )

        rospy.loginfo(f"Spawned {model_name} at ({x}, {y}, {z})")

    except Exception as e:
        rospy.logerr(f"Failed to spawn {model_name}: {e}")

if __name__ == '__main__':
    rospy.init_node('spawn_models_in_gazebo')

    # Paths to the model SDF files
    base_path = os.path.join(rospy.get_param("package_path", ""), "hma_hsr_sim_world/obj/ycb/")
    models = [
        ("orange", os.path.join(base_path, "017_orange/orange.sdf"), -0.3, 0.2, 0.5),
        ("soccer", os.path.join(base_path, "053_mini_soccer_ball/mini_soccer_ball.sdf"), 2.70, -0.8, 0.5),
        ("baseball", os.path.join(base_path, "055_baseball/baseball.sdf"), 2.70, -0.95, 0.5),
        ("softball", os.path.join(base_path, "054_softball/softball.sdf"), 2.70, -1.15, 0.5),
    ]

    for model_name, model_file, x, y, z in models:
        spawn_model_in_gazebo(model_name, model_file, x, y, z)

    rospy.spin()
