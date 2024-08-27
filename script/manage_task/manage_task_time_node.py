#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import threading
import subprocess

from std_msgs.msg import Bool


# Global
GP_LOOP_RATE = 10.0


class ManageTaskTime:
    """Manage task time."""
    
    def __init__(self, run_enable=True):
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        self.lib = {}

        self.run_enable = Bool(run_enable)

        # ROS I/F
        self.p_time = rospy.get_param(rospy.get_name() + "/time", 10.0)
    
        self.sub_run_enable = rospy.Subscriber(rospy.get_name() + "/run_enable",
                                               Bool,
                                               self.subf_run_enable,
                                               queue_size=1)

        return

    def delete(self):
        for key in self.lib.keys():
            self.lib[key].delete()
        return

    def subf_run_enable(self, run_enable):
        """Callback function for run enable.

        Args:
            run_enable (std_msgs/Bool): Run enable.
        """
        self.run_enable = run_enable
        if self.run_enable.data is True:
            rospy.loginfo("[" + rospy.get_name() + "]: Start.")
            rospy.loginfo("[" + rospy.get_name() + "]: TIME: " + str(self.p_time) + " minutes.")
        return


    def main(self):
        if self.run_enable.data == False:
            return

        rospy.sleep(self.p_time * 60.0)
        rospy.loginfo("[" + rospy.get_name() + "]: Finish.")
        
        rospy.loginfo("[" + rospy.get_name() + "]: Kill all ROS Nodes!!")
        _ = subprocess.run(
            "ps aux | grep ros | grep -v grep | awk \'{ print \"kill -9\", $2 }\' | sh",
            shell=True
        )
        
        sys.exit(1)
        return


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", GP_LOOP_RATE)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = ManageTaskTime(False)
    rospy.on_shutdown(cls.delete)
    while not rospy.is_shutdown():
        try:
            cls.main()
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        loop_wait.sleep()