import rospy
from sensor_msgs.msg import CameraInfo, Image
import cv_bridge, cv2
import numpy as np
from image_geometry import PinholeCameraModel

# Camera identifiers
LEFT_CAMERA = "rs_415_left"
RIGHT_CAMERA = "rs_415_right"

class Deprojection:
    def __init__(self) -> None:
        # Initialize camera-related variables
        self.left_depth_image = None
        self.left_camera_info = None
        self.left_color_image = None
        self.right_depth_image = None
        self.right_camera_info = None
        self.right_color_image = None
        self.cv_bridge = cv_bridge.CvBridge()
        self.left_camera_model = PinholeCameraModel()
        self.right_camera_model = PinholeCameraModel()
        
        # Camera topics
        left_camera_color_topic = f"/left/{LEFT_CAMERA}/color/image_raw"
        left_camera_info_topic = f"/left/{LEFT_CAMERA}/aligned_depth_to_color/camera_info"
        left_camera_depth_topic = f"/left/{LEFT_CAMERA}/aligned_depth_to_color/image_raw"
        right_camera_color_topic = f"/right/{RIGHT_CAMERA}/color/image_raw"
        right_camera_info_topic = f"/right/{RIGHT_CAMERA}/aligned_depth_to_color/camera_info"
        right_camera_depth_topic = f"/right/{RIGHT_CAMERA}/aligned_depth_to_color/image_raw"
        
        # Subscribers
        self.left_depth_image_sub = rospy.Subscriber(left_camera_depth_topic, Image, self.left_depth_image_callback)
        self.left_camera_info_sub = rospy.Subscriber(left_camera_info_topic, CameraInfo, self.left_camera_info_callback)
        self.left_color_image_sub = rospy.Subscriber(left_camera_color_topic, Image, self.left_color_image_callback)
        self.right_depth_image_sub = rospy.Subscriber(right_camera_depth_topic, Image, self.right_depth_image_callback)
        self.right_camera_info_sub = rospy.Subscriber(right_camera_info_topic, CameraInfo, self.right_camera_info_callback)
        self.right_color_image_sub = rospy.Subscriber(right_camera_color_topic, Image, self.right_color_image_callback)



        
    def __del__(self):
        del self.model

    def left_color_image_callback(self, msg: Image):
        self.left_color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def left_depth_image_callback(self, msg: Image):
        self.left_depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def left_camera_info_callback(self, msg: CameraInfo):
        self.left_camera_info = msg
        self.left_camera_model.fromCameraInfo(msg)

    def right_color_image_callback(self, msg: Image):
        self.right_color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def right_depth_image_callback(self, msg: Image):
        self.right_depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def right_camera_info_callback(self, msg: CameraInfo):
        self.right_camera_info = msg
        self.right_camera_model.fromCameraInfo(msg)

if __name__ == "__main__":
    rospy.init_node("deprojection_node")
    rospy.sleep(0.5)
    deproject = Deprojection()
    rospy.sleep(0.5)
    rospy.loginfo("Deproject server ready...")
    rospy.spin()