#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import roslib
import rospy
import threading
import cv2
import numpy
import datetime
import message_filters
import shutil

from cv_bridge import CvBridge
from gazebo_ros import gazebo_interface

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Image, CameraInfo


GP_DYNAMIC_LOOP_RATE = False
GP_LOOP_RATE = 30.0
camera_model_database_template = """<sdf version="1.4">
  <world name="default">
    <model name='task1_camera'>
      <static>true</static>
      <pose>-1.4411584138870239 -3.010014533996582 2.9286668300628662 0 0.7916429 1.5881942</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
        </visual>
        <sensor name='my_camera' type='camera'>
          <camera>
            <horizontal_fov>1.047</horizontal_fov>
            <image>
              <width>1920</width>
              <height>1080</height>
            </image>
            <clip>
              <near>0.1</near>
              <far>100</far>
            </clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
            <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>0.0</updateRate>
            <cameraName>camera</cameraName>
            <imageTopicName>image_raw</imageTopicName>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
            <frameName>camera_link</frameName>
            <hackBaseline>0.07</hackBaseline>
            <distortionK1>0.0</distortionK1>
            <distortionK2>0.0</distortionK2>
            <distortionK3>0.0</distortionK3>
            <distortionT1>0.0</distortionT1>
            <distortionT2>0.0</distortionT2>
        </plugin>
        </sensor>
      </link>
    </model>
  </world>
</sdf>"""

camera1_model_database_template = """<sdf version="1.4">
  <world name="default">
    <model name='task1_camera1'>
      <static>true</static>
      <pose>0.1 0.4 2.0 3.14 2.0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
        </visual>
        <sensor name='my_camera' type='camera'>
          <camera>
            <horizontal_fov>1.2</horizontal_fov>
            <image>
              <width>1920</width>
              <height>1080</height>
            </image>
            <clip>
              <near>0.1</near>
              <far>100</far>
            </clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
            <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>0.0</updateRate>
            <cameraName>camera1</cameraName>
            <imageTopicName>image_raw</imageTopicName>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
            <frameName>camera_link</frameName>
            <hackBaseline>0.07</hackBaseline>
            <distortionK1>0.0</distortionK1>
            <distortionK2>0.0</distortionK2>
            <distortionK3>0.0</distortionK3>
            <distortionT1>0.0</distortionT1>
            <distortionT2>0.0</distortionT2>
        </plugin>
        </sensor>
      </link>
    </model>
  </world>
</sdf>"""


class Shooting:
    def __init__(self, run_enable=True):
        """コンストラクタ.
        
        Args:
            run_enable (bool, optional): 実行許可
        """
        self._lock = threading.Lock()

        self._init_ros_time = rospy.Time.now()
        self._update_ros_time = {}
        self._prev_ros_time = self._init_ros_time

        self._lib = {}

        self._run_enable = Bool()
        self._image0 = Image()
        self._image1 = Image()
        self._cb = CvBridge()

        self._run_enable.data = run_enable
        
        self._my_pkg_dir = roslib.packages.get_pkg_dir("compe_pkg")
        self._save_path = self._my_pkg_dir + "/io/images"
        rospy.loginfo(self._save_path)
        self._cnt = 0

        # make save folder
        camera_cnt = 2
        create_datetime = datetime.datetime.now() 
        self.datetime_str = str(create_datetime.year) + "-" + \
                            str(create_datetime.month).rjust(2, "0")  + "-" + \
                            str(create_datetime.day).rjust(2, "0")    + "_" + \
                            str(create_datetime.hour).rjust(2, "0")   + ":" + \
                            str(create_datetime.minute).rjust(2, "0") + ":" + \
                            str(create_datetime.second).rjust(2, "0")
        for i in range(camera_cnt):
            save_path = self._save_path + "/" + self.datetime_str + "/" + str(i) + "/"
            if not os.path.exists(save_path):
                os.makedirs(save_path)
                rospy.loginfo("Make save file. : " + save_path)
            else:
                rospy.loginfo("Exists save file.")
        
        # make csv file
        csv_file_path = self._save_path + "/" + self.datetime_str + "/" + "data.csv"
        with open(csv_file_path, 'w') as file:
            file.write('TeamName:\n')
            file.write('frame,score,hit,drop,object_id\n')
            file.write('0,0,0,0,0\n')
        make_video_py_path = f"{self._my_pkg_dir}/script/manage_task/shooting/make_video_py.py"
        
        # copy make_video_py.py
        dst_path = self._save_path + "/" + self.datetime_str + "/"
        shutil.copy(make_video_py_path, dst_path)
        

        # ROSインタフェース
        self._pub_spawn_camera_state = rospy.Publisher(
            "/shooting_node/spawn_camera/spawned", Bool, queue_size=1)
        
        self._sub_run_enable = rospy.Subscriber(
            "/manage_task_time_node/run_enable", Bool, self.subfRunEnable, queue_size=1)

        self._sub_image0 = message_filters.Subscriber(
            "/camera/image_raw", Image)

        self._sub_image1 = message_filters.Subscriber(
            "/camera1/image_raw", Image)

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._sub_image0, self._sub_image1],
            1,  # Queueサイズ
            0.1,  # 許容時間ずれ
            allow_headerless=True
        )
        self._sync.registerCallback(self.subfImage)
        
        self.is_spawn_camera = False
        self._sub_spawn_camera_run_enable = rospy.Subscriber(
            "/shooting_node/spawn_camera/run_enable", Bool, self.sub_spawn_camera, queue_size=1)
        
        return

    def delete(self):
        """デストラクタ."""
        for key in self._lib.keys():
            self._lib[key].delete()

        return
    
    def sub_spawn_camera(self, run_enable):
        """カメラ召喚許可サブスクライブ関数.
        
        Args:
            run_enable (Bool): 実行許可
        """
        if self._run_enable.data is False:
            if run_enable:
              rospy.loginfo("[" + rospy.get_name() + "]: SPAWN CAMERA!!")
              self.is_spawn_camera = True
              self.drop_camera()
              self._pub_spawn_camera_state.publish(True)
              rospy.sleep(1)
        return
    
    def subfRunEnable(self, run_enable):
        """実行許可サブスクライブ関数.
        
        Args:
            run_enable (Bool): 実行許可
        """
        self._run_enable = run_enable

        if self._run_enable.data is True and self.is_spawn_camera is True:
            rospy.loginfo("[" + rospy.get_name() + "]: SHOOTING START!!")

        return

    def subfImage(self, image0, image1):
        """Imageサブスクライブ関数.
        
        Args:
            image0 (Image()): 入力画像
            image1 (Image()): 入力画像
        """
        try:
            self.proc(image0, image1)
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        return

    def convSMI2CV(self, smi, encode, is_depth=False):
        """smiからcvに変換する関数.
        
        Args:
            smi (Image()): 画像（SMI）
            encode (str): 画像エンコード
            is_depth (bool, optional): depth画像判定
        
        Returns:
            array or int: 画像 or 1(失敗時)
        """
        try:
            if is_depth:
                cv_tmp = self._cb.imgmsg_to_cv2(smi, encode)
                cv = numpy.array(cv_tmp, dtype=numpy.uint16)
            else:
                cv = self._cb.imgmsg_to_cv2(smi, encode)
        except:
            rospy.logwarn("[" + rospy.get_name() + "]: CV bridge FAILURE")
            return 1
        return cv

    def drop_camera(self):
        rospy.loginfo('Drop {0}'.format("task1_camera"))
        initial_pose = Pose()
        initial_pose.position.x = -1.4411584138870239
        initial_pose.position.y = -3.010014533996582
        initial_pose.position.z = 2.9286668300628662
        initial_pose.orientation = Quaternion(
            -0.27499783039093018,
            0.27025458216667175,
            0.65808409452438354,
            0.646733283996582
        )

        model_xml = camera_model_database_template.replace('MODEL_NAME', "task1_camera")

        gazebo_interface.spawn_sdf_model_client("camera", model_xml, rospy.get_namespace(),
                                                initial_pose, "", "/gazebo")

        rospy.loginfo('Drop {0}'.format("task1_camera1"))

        model_xml = camera1_model_database_template.replace('MODEL_NAME', "task1_camera1")

        gazebo_interface.spawn_sdf_model_client("camera", model_xml, rospy.get_namespace(),
                                                initial_pose, "", "/gazebo")

    def proc(self, image0, image1):
        """処理関数.
        
        Args:
            image0 (Image): 入力画像
            image1 (Image): 入力画像
        """
        # 実行許可の検証
        if self._run_enable.data == False:
            return
        
        # proc
        cv_img0 = self.convSMI2CV(image0, "bgr8")
        cv_img1 = self.convSMI2CV(image1, "bgr8")
        cv2.imwrite(self._save_path + "/" + self.datetime_str + "/0/" + str(self._cnt).zfill(7) + ".jpg", cv_img0)
        cv2.imwrite(self._save_path + "/" + self.datetime_str + "/1/" + str(self._cnt).zfill(7) + ".jpg", cv_img1)
        
        self._cnt += 1
        # cv2.imshow("test", cv_img)
        # cv2.waitKey(1)
        return


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", GP_LOOP_RATE)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = Shooting(False)

    rospy.on_shutdown(cls.delete)
    
    while not rospy.is_shutdown():
        try:
            # cls.proc()
            pass
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        loop_wait.sleep()
