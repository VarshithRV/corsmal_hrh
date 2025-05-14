# apriltag grid detection for realsense type camera
import rospy
from sensor_msgs.msg import CameraInfo, Image
import cv_bridge, cv2
import numpy as np
from image_geometry import PinholeCameraModel
import apriltag
import copy
import threading
from geometry_msgs.msg import PoseStamped, Pose
import quaternion
import tf.transformations as tft
import sys
from scipy.spatial.transform import Rotation as R


class Deprojection:
    def __init__(self) -> None:
        self.depth_image = None
        self.camera_info = None
        self.grayscale_image = None
        self.cv_bridge = cv_bridge.CvBridge()
        self.camera_model = PinholeCameraModel()
        self.detection_rate = 30
        self.estimated_pose = None
        self.filtered_pose = None
        self.rvec = None
        self.tvec = None
        self.tvec_impulse = None
        self.filtered_tvec = np.zeros(shape=(3,),dtype=float)
        self.qvec = None
        self.qvec_impulse = None
        self.filtered_qvec = np.array([0,0,0,1],dtype=float)
        self.mutex = threading.Lock()
        self.detected_image = None

        # linear infinite impulse response filter parameters
        self.alpha = rospy.get_param("~alpha",0.25)
        
        self.object_name = rospy.get_param("~object/name","object0")
        
        # Camera topics
        camera_color_topic = rospy.get_param("~color_image_topic","/camera/color/image_raw")
        camera_info_topic = rospy.get_param("~camera_info_topic","/camera/color/image_raw")
        camera_depth_topic = rospy.get_param("~depth_image_topic","/camera/color/image_raw")

        detected_image_topic = f"/detected_image_raw"
        
        rospy.loginfo("Mediapipe ros node started with params : ")
        rospy.loginfo("Alpha : %s"%self.alpha)
        rospy.loginfo("Object Name : %s"%self.object_name)
        rospy.loginfo("Color image topic : %s"%camera_color_topic)
        rospy.loginfo("Camera info topic : %s"%camera_info_topic)
        rospy.loginfo("Depth image topic : %s"%camera_depth_topic)


        # Pose topics
        pose_topic = f"/{self.object_name}_pose"
        filtered_pose_topic = f"/{self.object_name}_filtered_pose"
        
        # Subscribers
        self.depth_image_sub = rospy.Subscriber(camera_depth_topic, Image, self.depth_image_callback)
        self.camera_info_sub = rospy.Subscriber(camera_info_topic, CameraInfo, self.camera_info_callback)
        self.color_image_sub = rospy.Subscriber(camera_color_topic, Image, self.color_image_callback)

        # Publishers
        self.detected_image_pub = rospy.Publisher(detected_image_topic, Image, queue_size=10)
        self.pose_pub = rospy.Publisher(pose_topic,PoseStamped,queue_size=10)
        self.filtered_pose_pub = rospy.Publisher(filtered_pose_topic,PoseStamped,queue_size=10)

        # Timers
        self.apriltag_detector_timer = rospy.Timer(rospy.Duration(1/self.detection_rate),callback=self.apriltag_detector_cb)
        self.filtered_pose_publisher_timer = rospy.Timer(rospy.Duration(1/self.detection_rate),callback=self.filtered_pose_publisher_cb)
        self.pose_publisher_timer = rospy.Timer(rospy.Duration(1/self.detection_rate),callback=self.pose_publisher_cb)
        self.detected_image_publisher_timer = rospy.Timer(rospy.Duration(1/self.detection_rate),callback=self.detected_image_publisher_cb)

    def __del__(self):
        del self.camera_model

    def draw_axes(self,image, rvec, tvec, intrinsics, distortion_coeffs, axis_length=0.05):
        # Define the axes in 3D space (in the object coordinate frame)
        axis_3d = np.float32([
            [0, 0, 0],# origin
            [axis_length, 0, 0],# X axis (red)
            [0, axis_length, 0],# Y axis (green)
            [0, 0, axis_length]# Z axis (blue)
        ]).reshape(-1, 3)

        # Project 3D points to image plane
        imgpts, _ = cv2.projectPoints(axis_3d, rvec, tvec, intrinsics, distortion_coeffs)

        imgpts = imgpts.astype(int)

        origin = tuple(imgpts[0].ravel())
        x_axis = tuple(imgpts[1].ravel())
        y_axis = tuple(imgpts[2].ravel())
        z_axis = tuple(imgpts[3].ravel())

        # Draw the axes
        cv2.line(image, origin, x_axis, (0, 0, 255), 2)   # X in red
        cv2.line(image, origin, y_axis, (0, 255, 0), 2)   # Y in green
        cv2.line(image, origin, z_axis, (255, 0, 0), 2)   # Z in blue
        cv2.circle(image, origin, 3, (0, 0, 0), -1)       # Draw origin point

        return image

    def apriltag_detector_cb(self,event):
        if self.grayscale_image is None : 
            return
        image_detected,rvec,tvec = self.detect_hands(self.grayscale_image)
        self.detected_image = image_detected
        
        if tvec is not None or rvec is not None:
            self.tvec = tvec
            # rotation_matrix, _ = cv2.Rodrigues(self.rvec)
            r = R.from_rotvec(rvec)
            self.qvec = r.as_quat()
        else :
            self.tvec = None
            self.qvec = None
    
    def detected_image_publisher_cb(self,event):
        if self.detected_image is not None:
            image = self.cv_bridge.cv2_to_imgmsg(cvim=self.detected_image,encoding="bgr8")
            self.detected_image_pub.publish(image)

    def pose_publisher_cb(self,event):
        if self.qvec is None or self.tvec is None : 
            return

        msg = PoseStamped()
        msg.header.frame_id = self.camera_info.header.frame_id
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = self.tvec[0]
        msg.pose.position.y = self.tvec[1]
        msg.pose.position.z = self.tvec[2]
        msg.pose.orientation.x = self.qvec[0]
        msg.pose.orientation.y = self.qvec[1]
        msg.pose.orientation.z = self.qvec[2]
        msg.pose.orientation.w = self.qvec[3]

        self.pose_pub.publish(msg)        

    def filter_pose(self):
        # filter self.tvec and self.qvec with some alpha and update self.filtered_tvec and self.filtered_qvec
        q1 = self.filtered_qvec
        q2 = self.qvec

        dot = np.dot(q1,q2)
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        if dot > 0.9995:
            # Linear interpolation
            filtered_quat = (self.alpha)*q1 + (1-self.alpha)*q2
        else : 
            theta_0 = np.arccos(dot)
            sin_theta_0 = np.sin(theta_0)
            theta = theta_0*(1-self.alpha)
            sin_theta = np.sin(theta)

            s1 = np.sin(theta_0 - theta) / sin_theta_0
            s2 = sin_theta / sin_theta_0

            filtered_quat = s1 * q1 + s2 * q2
        filtered_quat = filtered_quat / np.linalg.norm(filtered_quat)
        #SLERP for qvec
        self.filtered_qvec = filtered_quat
        #LERP for tvec
        self.filtered_tvec = self.alpha*self.filtered_tvec + (1-self.alpha)*self.tvec

    def filtered_pose_publisher_cb(self,event):
        if self.tvec is None or self.qvec is None :
            return
        
        self.filter_pose()
        msg = PoseStamped()
        msg.header.frame_id = self.camera_info.header.frame_id
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = self.filtered_tvec[0]
        msg.pose.position.y = self.filtered_tvec[1]
        msg.pose.position.z = self.filtered_tvec[2]
        msg.pose.orientation.x = self.filtered_qvec[0]
        msg.pose.orientation.y = self.filtered_qvec[1]
        msg.pose.orientation.z = self.filtered_qvec[2]
        msg.pose.orientation.w = self.filtered_qvec[3]
        self.filtered_pose_pub.publish(msg)

    def detect_hands(self,grayscale_image):
        # need to detect stuff here, draw_axes on the image, return detected_image, rvec in (3,) and tvec in (3,)
        detected_image = cv2.cvtColor(copy.deepcopy(grayscale_image),cv2.COLOR_GRAY2RGB)
        
        # detect image, and find rvec and tvec here : 
        
        if self.camera_model is not None : 
            intrinsics = self.camera_model.K
            distortion_coefficients = self.camera_model.D
            unit_rotation_mat = np.eye(3,3,dtype=float)
            rvec = np.zeros((3,),dtype=float)
            rvec, _ = cv2.Rodrigues(unit_rotation_mat)
            tvec = np.array([0,0,0],dtype = float)
            detected_image = self.draw_axes(detected_image,rvec,tvec,intrinsics,distortion_coefficients)
        return detected_image, None, None

    def display_color_image(self, event):
        if self.grayscale_image is not None :
            cv2.imshow("annotated_image",self.grayscale_image)
            cv2.waitKey(1)

    def color_image_callback(self, msg: Image):
        self.grayscale_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

    def depth_image_callback(self, msg: Image):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg
        self.camera_model.fromCameraInfo(msg)

if __name__ == "__main__":
    rospy.init_node("mediapipe_ros_driver",argv=sys.argv,anonymous=False)
    rospy.sleep(0.15)
    deproject = Deprojection()
    rospy.loginfo("Detector running")
    rospy.spin()