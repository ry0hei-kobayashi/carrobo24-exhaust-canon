import rospy
import smach
from tam_object_detection.srv import ObjectDetectionService, ObjectDetectionServiceRequest
from tamlib.utils import Logger

class Recog(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes, input_keys=["detected_object"], output_keys=["detected_object"])
        Logger.__init__(self)

        # Service
        self.srv_detection = rospy.ServiceProxy(
            "hsr_head_rgbd/object_detection/service", ObjectDetectionService
        )
        rospy.wait_for_service("hsr_head_rgbd/object_detection/service", timeout=10)

    def execute(self, userdata):
        # Object detection request
        det_req = ObjectDetectionServiceRequest(use_latest_image=True)
        detections = self.srv_detection(det_req).detections

        if detections.is_detected is False:
            self.logwarn("No object detected.")
            return "failure"
        
        userdata.detected_object = []

        # Log the detected object information
        for i in range(len(detections.bbox)):
            label = detections.bbox[i].name
            if label == 'toy_airplane':
                continue

            userdata.detected_object.append({
                "bbox": detections.bbox[i],
                "pose": detections.pose[i]
            })
            # sm.userdata.detected_object.append(detections.is_detected)
            # sm.userdata.detected_object.append(detections.depth)
            # sm.userdata.detected_object.append(detections.rgb)
            # sm.userdata.detected_object.append(detections.camera_info)
            # sm.userdata.detected_object.append(detections.segments)


            self.loginfo(f"Object {i}:")
            self.loginfo(f"  Label: {label}")

            bbox = detections.bbox[i]
            self.loginfo(f"  Bounding Box: x={bbox.x}, y={bbox.y}, width={bbox.w}, height={bbox.h}")

            pose = detections.pose[i]
            self.loginfo(f"  Pose: position={pose.position}, orientation={pose.orientation}")

        return "next"
