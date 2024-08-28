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

    #def calculate_euclidean_distance(self, pose):
    #    # Assuming pose.position is a geometry_msgs/Point-like object with attributes x, y, z
    #    # return math.sqrt(pose.position.x ** 2 + pose.position.z ** 2)
    #    return pose.position.z

    def execute(self, userdata):
        # Object detection request
        self.hsrif.whole_body.move_to_joint_positions(
            {
                #"arm_lift_joint": 0.0,
                #"arm_flex_joint": np.deg2rad(0.0),
                #"arm_roll_joint": np.deg2rad(90.0),
                #"wrist_roll_joint": np.deg2rad(0.0),
                #"wrist_flex_joint": np.deg2rad(-110.0),
                #"head_pan_joint": 0.0,
                "head_tilt_joint": np.deg2rad(-52.0),
            }, 
        )

        #self.hsrif.whole_body.move_to_joint_positions(
        #    {
        #        "head_tilt_joint": np.deg2rad(-52),
        #    }(,( 
        #    sync=True
        #)
        rospy.sleep(1)
        det_req = ObjectDetectionServiceRequest(use_latest_image=True, max_distance=0.9)
        detections = self.srv_detection(det_req).detections

        self.loginfo('認識結果')
        self.loginfo(len(detections.bbox))
        if not detections.is_detected:
            userdata.position += 1
            self.loginfo(detections.bbox)
            self.logwarn("No object detected.")
            return "failure"
        
        userdata.detected_obj = []

        min_dist = np.inf
        obj_idx = None
        x_min ,x_max = -0.5, 0.5
        y_min ,y_max = -0.5, 0.5

        # Log the detected object information and append to userdata
        rospy.logwarn('recog.-> will find nearness obj')
        for i in range(len(detections.bbox)):

            if detections.pose[i].is_valid is False: #can not pose detection
                rospy.logwarn('recog.-> detection pose is valid')
                continue

            label = detections.bbox[i].name
            if label == 'toy_airplane': #retire difficult obj
                rospy.logwarn('recog.-> skip difficult obj')
                continue

            x = detections.pose[i].position.x
            y = detections.pose[i].position.y
            z = detections.pose[i].position.z

            dist = np.sqrt(x ** 2 + y ** 2 + z ** 2)

            if not (x_min < x_max and y_min < y_max):
                continue

            userdata.depth = detections.depth
            userdata.detected_obj.append({
                "bbox": detections.bbox[i],
                "pose": detections.pose[i],
                "seg": detections.segments[i],
                "distance": dist,
                })



        userdata.detected_obj.sort(key=lambda obj: obj["distance"])

        #if obj_idx is None:
        #    rospy.logwarn('recog.-> cannot find nearness obj.-> loop'
        #    #userdata location
        #    #return "goto"



        for i, obj in enumerate(userdata.detected_obj):
            self.loginfo(f"  label = {obj['bbox'].name}")
            self.loginfo(f"  Bounding Box: x={obj['bbox'].x}, y={obj['bbox'].y}, width={obj['bbox'].w}, height={obj['bbox'].h}")
            self.loginfo(f"  Pose: position=({obj['pose'].position.x}, {obj['pose'].position.y}, {obj['pose'].position.z}), orientation=({obj['pose'].orientation.x}, {obj['pose'].orientation.y}, {obj['pose'].orientation.z}, {obj['pose'].orientation.w})")
            self.loginfo(f"  Distance: {obj['distance']}")

        return "next"

