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
        self.sm.userdata.depth = []
        self.sm.userdata.grasp_counter = 0
        self.sm.userdata.position = 0
        self.sm.userdata.search_locations = {
            #key:{x,y,yaw}
            0: (1.0, 2.0, 1.57  ), 
            1: (2.0, 1.0, -1.57 ), 
            2: (.0, .0, 1.57    ), 
            3: (.0, .0, 1.57    ) 
            }
        self.sm.deposit_locations = {
            #key:{x,y,yaw}
            'kitchen':     (5.0, 5.0, 1.57  ), 
            'tool': (2.0, 1.0, -1.57 ), 
            'task': (2.0, 1.0, -1.57 ), 
            'food':  (.0, .0, 1.57    ), 
            'shape':  (.0, .0, 1.57    ),
            'orientation':  (.0, .0, 1.57    ), 
            'unknown':  (.0, .0, 1.57    ),
            }

        self.sm.userdata.locations = None

        with self.sm:
            smach.StateMachine.add(
                "Init",
                standard.Init(["next"]),
                transitions={"next": "GoToFloor"},
            )
            smach.StateMachine.add(
                "GoToFloor",
                goto.GoToFloor(["next"]),
                transitions={"next": "Recog"},
            )
            smach.StateMachine.add(
                "Recog",
                recog.Recog(["success", "failure"]),
                transitions={
                    "success": "GraspFromFloor",
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
                deposit.DepositObject(["next", "re_recog"]),
                transitions={
                    "next": "GraspFromFloor", 
                    "re_recog": "GoToFloor",
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

