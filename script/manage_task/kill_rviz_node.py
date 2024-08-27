#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import rospy

class KillRvizNode:
    def __init__(self):
        rospy.init_node('kill_rviz_node')
        
    def kill_rviz(self):
        command = "kill $(pgrep rviz | head -n 1)"
        subprocess.call(command, shell=True)

def main():
    node = KillRvizNode()
    node.kill_rviz()

if __name__ == '__main__':
   main() 