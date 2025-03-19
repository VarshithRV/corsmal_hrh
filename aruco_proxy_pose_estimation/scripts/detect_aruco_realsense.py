import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from cv2 import aruco

class ArucoDetector:
    def __init__(self):
        rospy.init_node("aruco_detector", anonymous=True)
        
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_size = 0.04  # Marker size in meters (adjust accordingly)
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        self.aruco_params = aruco.DetectorParameters()
        
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        
        self.image_pub = rospy.Publisher("/aruco_detected_image", Image, queue_size=1)
        rospy.loginfo("Aruco Detector Node Started")
    
    def camera_info_callback(self, msg:CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            rospy.loginfo("Camera Info Received")
    
    def image_callback(self, msg:Image):
        if self.camera_matrix is None:
            rospy.logwarn("Waiting for camera info...")
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
            return
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        corners, ids, _ = detector.detectMarkers(gray)
        
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in [21,0,5]:
                    aruco.drawDetectedMarkers(cv_image, [corners[i]], np.array([[marker_id]]))
        
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_pub.publish(ros_image)
        
        cv2.imshow("Aruco Detection", cv_image)
        cv2.waitKey(1)
    
if __name__ == "__main__":
    try:
        ArucoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
