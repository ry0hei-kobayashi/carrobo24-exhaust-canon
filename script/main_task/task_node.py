#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import os

import rospy
import smach
import smach_ros
from hsrlib.hsrif import HSRInterfaces
from state_machine import (
    deposit,
    goto,
    #grasp,
    standard,
)


class StateMachine:
    def __init__(self) -> None:
        self.hsrif = HSRInterfaces()

        # ステートマシンの宣言
        self.sm = smach.StateMachine(outcomes=["exit"])
        self.sm.userdata.a=1
        self.sm.userdata.list=[]
        with self.sm:
            smach.StateMachine.add(
                "Init",
                standard.Init(["next"]),
                transitions={"next": "GoToFloor"},
            )
            smach.StateMachine.add(
                "GoToFloor",
                goto.GoToFloor(["next"]),
                #transitions={"next": "GraspFromFloor"},
                transitions={"next": "DepositObject"}
            )
            #smach.StateMachine.add(
            #    "GraspFromFloor",
            #    grasp.GraspFromFloor(["next", "failure"]),
            #    transitions={
            #        "next": "DepositObject",
            #        "failure": "GoToFloor",
            #    },
            #)
            smach.StateMachine.add(
                "DepositObject",
                deposit.DepositObject(["next"]),
                transitions={"next": "GoToFloor", },
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
