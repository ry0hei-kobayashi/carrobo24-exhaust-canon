#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import os

import rospy
import smach
import smach_ros
from hsrlib.hsrif import HSRInterfaces
from state_machine import (
    recog,
    deposit,
    goto,
    recog,
    grasp,
    standard,
)


class StateMachine:
    def __init__(self) -> None:
        self.hsrif = HSRInterfaces()

        # ステートマシンの宣言
        self.sm = smach.StateMachine(outcomes=["exit"])

        self.sm.userdata.detected_obj = []
        self.sm.userdata.depth = None
        self.sm.userdata.grasp_counter = 0
        self.sm.userdata.position = 0
        self.sm.userdata.search_locations = {
            #key:{x,y,yaw}
            0: (0.74, 0.70, 1.57  ), #中央 
            1: (0.54, 0.70,   1.3 ), 
            2: (0.14, 0.70, 1.57),
            3: (-0.34, 0.70,     1.57    ),
            4: (-0.54, 0.70,     1.57    ), #一番手前
            }
        self.sm.userdata.deposit_locations = {
            #key:{x,y,yaw}
            'kitchen':     (0.95, -0.23, -1.57  ), 
            'tool':        (-0.14, 0.22, -1.57 ),  #right
            'task':        (2.30, -0.15, -1.57 ), 
            'food1':       (1.53, -0.20, -1.57    ), #right #TODO 増やして，randomでfoodxを回したい
            'food2':       (1.66, -0.20, -1.57    ), #left
            #'food3':       (1.23, -0.20, -1.57    ), #left
            'shape':       (0.14, 0.22, -1.57    ), #left
            'orientation': (1.15, -0.20, -1.57    ), 
            'unknown':     (2.74, -0.15, -1.57    ),
            }

        # self.sm.userdata.locations = None
        self.sm.userdata.food_select = 1

        with self.sm:
            smach.StateMachine.add(
                "Init",
                standard.Init(["next"]),
                transitions={"next": "GoToFloor"},
            )
            smach.StateMachine.add(
                "GoToFloor",
                goto.GoToFloor(["recog", "grasp"]),
                transitions={"recog": "Recog",
                            "grasp": "GraspFromFloor"},
            )
            smach.StateMachine.add(
                "Recog",
                recog.Recog(["next", "failure"]),
                transitions={
                    "next": "GraspFromFloor",
                    "failure": "GoToFloor",
                },
            )
            smach.StateMachine.add(
                "GraspFromFloor",
                grasp.GraspFromFloor(["next", "failure", "nothing"]),
                transitions={
                    "next": "DepositObject",
                    "failure": "GraspFromFloor",
                    "nothing" : "GoToFloor",
                },
            )
            smach.StateMachine.add(
                "DepositObject",
                deposit.DepositObject(["next"]),
                transitions={
                    "next": "GoToFloor", 
                },
            )
            smach.StateMachine.add(
                "Finish",
                standard.Finish(["finish"]),
                transitions={"finish": "exit"},
            )
        rospy.loginfo("sm add end")

        sris = smach_ros.IntrospectionServer("sm", self.sm, "/SM_ROOT")
        sris.start()

    def delete(self) -> None:
        del self.sm

    def run(self) -> None:
        self.sm.execute()


def main():
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    cls = StateMachine()
    rospy.on_shutdown(cls.delete)
    try:
        cls.run()
    except rospy.exceptions.ROSException as e:
        rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        rospy.logerr("[" + rospy.get_name() + "]: " + str(e))


if __name__ == "__main__":
    main()

