import rospy
import smach
from tam_object_detection.srv import ObjectDetectionService, ObjectDetectionServiceRequest
from tamlib.utils import Logger
from hsrlib.hsrif import HSRInterfaces
import numpy as np
import math

class Recog(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes, input_keys=["detected_obj", 'position', 'depth'], output_keys=["detected_obj", 'depth', 'position'])
        Logger.__init__(self)

        # Service
        self.srv_detection = rospy.ServiceProxy(
            "hsr_head_rgbd/object_detection/service", ObjectDetectionService
        )
        rospy.wait_for_service("hsr_head_rgbd/object_detection/service", timeout=1000)

        self.hsrif = HSRInterfaces()

    def calculate_euclidean_distance(self, pose):
        # Assuming pose.position is a geometry_msgs/Point-like object with attributes x, y, z
        # return math.sqrt(pose.position.x ** 2 + pose.position.z ** 2)
        return pose.position.z

    def execute(self, userdata):
        # Object detection request
        self.hsrif.whole_body.move_to_joint_positions(
            {
                "arm_lift_joint": 0.0,
                "arm_flex_joint": np.deg2rad(0.0),
                "arm_roll_joint": np.deg2rad(90.0),
                "wrist_roll_joint": np.deg2rad(0.0),
                "wrist_flex_joint": np.deg2rad(-110.0),
                "head_pan_joint": 0.0,
                "head_tilt_joint": np.deg2rad(-40),
            }, 
            sync=True
        )
        det_req = ObjectDetectionServiceRequest(use_latest_image=True)
        detections = self.srv_detection(det_req).detections

        self.loginfo('認識結果')
        self.loginfo(len(detections.bbox))
        if detections.is_detected is False:
            userdata.position += 1
            self.loginfo(detections.bbox)
            self.logwarn("No object detected.")
            return "failure"



        # Log the detected object information and append to userdata
        for i in range(len(detections.bbox)):
            label = detections.bbox[i].name
            if label == 'toy_airplane':
                continue

            distance = self.calculate_euclidean_distance(detections.pose[i])

            userdata.depth = detections.depth
            userdata.detected_obj.append({
                "bbox": detections.bbox[i],
                "pose": detections.pose[i],
                "seg": detections.segments[i],
                "distance": distance,
            })

        # Sort the objects by distance (ascending) in userdata.detected_object
        userdata.detected_obj.sort(key=lambda obj: obj["distance"])

        # Log sorted objects
        # self.loginfo("Objects sorted by distance:")
        for i, obj in enumerate(userdata.detected_obj):
            # self.loginfo(f"Object {i}:")
            # self.loginfo(f"Label: {obj['label']}")
            # bbox = obj['bbox']
            # self.loginfo(f"  Bounding Box: x={bbox.x} y={bbox.y} width={bbox.w} height={bbox.h}")
            pose = obj['pose']
            self.loginfo(f"  Pose: position={pose.position}, orientation={pose.orientation}")
            self.loginfo(f"  Distance: {obj['distance']}")

        return "next"
