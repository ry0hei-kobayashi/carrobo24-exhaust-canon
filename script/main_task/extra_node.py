#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import os

import rospy
import smach
import smach_ros
from hsrlib.hsrif import HSRInterfaces
from state_machine import (
    sound_recog,
    extra_goto,
    extra_grasp,
)


class StateMachine:
    def __init__(self) -> None:
        self.hsrif = HSRInterfaces()

        self.sm = smach.StateMachine(outcomes=["exit"])
        self.sm.userdata.places = {
            #key:{x,y,yaw}
            "table": (0.74, 0.9, 1.57  ),
            "shelf": (0.74, 0.9, 1.57  ),
            # TODO: table shelf iti
            # TODO: hito iti
            }
        self.sm.userdata.object=""
        self.sm.userdata.place=""
        with self.sm:
            smach.StateMachine.add(
                # 音声認識して、掴む場所・物体名を伝える
                "SoundRecog",
                sound_recog.LibSpeechRecog(["next", "loop"]),
                transitions={
                    "next": "GoObj",
                    "loop": "SoundRecog",
                },
            )
            smach.StateMachine.add(
                # オブジェクトの場所に
                "GoObj",
                extra_goto.GoToFloor(["next", "failure"]),
                transitions={
                    "next": "GetObj",
                    "failure": "GoObj",
                },
            )
            smach.StateMachine.add(
                # オブジェクトを取る
                "GetObj",
                deposit.DepositObject(["next", "loop"]),
                transitions={"next": "MovePerson", "loop": "GetObj"},
            )
            smach.StateMachine.add(
                # 人を見つける
                "MovePerson",
                deposit.DepositObject(["next"]),
                transitions={"next": "exit", },
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
