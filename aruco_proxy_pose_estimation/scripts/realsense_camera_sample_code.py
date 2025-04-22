# sample code for realsense type camera
# cv_bridge documentation : https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
import rospy
from sensor_msgs.msg import CameraInfo, Image
import cv_bridge, cv2
import numpy as np
from image_geometry import PinholeCameraModel

class Deprojection:
    def __init__(self) -> None:
        self.depth_image = None
        self.camera_info = None
        self.color_image = None
        self.cv_bridge = cv_bridge.CvBridge()
        self.camera_model = PinholeCameraModel()
        
        # Camera topics
        camera_color_topic = f"/camera/color/image_raw"
        camera_info_topic = f"/camera/aligned_depth_to_color/camera_info"
        camera_depth_topic = f"/camera/aligned_depth_to_color/image_raw"
        
        # Subscribers
        self.depth_image_sub = rospy.Subscriber(camera_depth_topic, Image, self.depth_image_callback)
        self.camera_info_sub = rospy.Subscriber(camera_info_topic, CameraInfo, self.camera_info_callback)
        self.color_image_sub = rospy.Subscriber(camera_color_topic, Image, self.color_image_callback)

        # Timers
        self.display_color_image_timer = rospy.Timer(rospy.Duration(0.05),callback=self.display_color_image)
        
    def __del__(self):
        del self.camera_model

    def display_color_image(self, event):
        if self.color_image is not None :
            cv2.imshow("annotated_image",self.color_image)
            cv2.waitKey(1)

    def color_image_callback(self, msg: Image):
        self.color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="mono16") # change to bgr8 or rgb8 for color

    def depth_image_callback(self, msg: Image):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg
        self.camera_model.fromCameraInfo(msg)

if __name__ == "__main__":
    rospy.init_node("realsense_sample_node")
    rospy.sleep(0.15) # some time to initialize stuff
    deproject = Deprojection()
    rospy.loginfo("Node running")
    rospy.spin()