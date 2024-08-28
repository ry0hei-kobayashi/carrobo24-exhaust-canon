import rospy
import smach
from tam_object_detection.srv import ObjectDetectionService, ObjectDetectionServiceRequest
from tamlib.utils import Logger
import math

class Recog(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes, input_keys=["detected_object"], output_keys=["detected_object"])
        Logger.__init__(self)

        # Service
        self.srv_detection = rospy.ServiceProxy(
            "hsr_head_rgbd/object_detection/service", ObjectDetectionService
        )
        rospy.wait_for_service("hsr_head_rgbd/object_detection/service", timeout=10)

    def calculate_euclidean_distance(self, pose):
        # Assuming pose.position is a geometry_msgs/Point-like object with attributes x, y, z
        return math.sqrt(pose.position.x ** 2 + pose.position.z ** 2)

    def execute(self, userdata):
        # Object detection request
        det_req = ObjectDetectionServiceRequest(use_latest_image=True)
        detections = self.srv_detection(det_req).detections

        if not detections.is_detected:
            self.logwarn("No object detected.")
            return "failure"

        # Clear the userdata detected_object list
        userdata.detected_object = []

        # Log the detected object information and append to userdata
        for i in range(len(detections.bbox)):
            label = detections.bbox[i].name
            if label == 'toy_airplane':
                continue

            # Calculate distance
            distance = self.calculate_euclidean_distance(detections.pose[i])
            
            # Create object info
            obj_info = {
                "label": label,
                "bbox": detections.bbox[i],
                "pose": detections.pose[i],
                "distance": distance,
                
            }
            userdata.detected_object.append(obj_info)

            # # Log object information
            # self.loginfo(f"Object {i}:")
            # self.loginfo(f"  Label: {label}")
            
            # bbox = detections.bbox[i]
            # self.loginfo(f"  Bounding Box: x={bbox.x}, y={bbox.y}, width={bbox.w}, height={bbox.h}")
            
            # pose = detections.pose[i]
            # self.loginfo(f"  Pose: position={pose.position}, orientation={pose.orientation}")
            
            # # Log distance
            # self.loginfo(f"  Distance: {distance}")

        # Sort the objects by distance (ascending) in userdata.detected_object
        userdata.detected_object.sort(key=lambda obj: obj["distance"])

        # Log sorted objects
        self.loginfo("Objects sorted by distance:")
        for i, obj in enumerate(userdata.detected_object):
            self.loginfo(f"Object {i}:")
            self.loginfo(f"Label: {obj['label']}")
            bbox = obj['bbox']
            self.loginfo(f"  Bounding Box: x={bbox.x} y={bbox.y} width={bbox.w} height={bbox.h}")
            pose = obj['pose']
            self.loginfo(f"  Pose: position={pose.position}, orientation={pose.orientation}")
            self.loginfo(f"  Distance: {obj['distance']}")

        return "next"
