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
import tf2_geometry_msgs
import quaternion
import tf.transformations as tft
import mediapipe as mp
import tf,tf2_ros
import sys
from scipy.spatial.transform import Rotation as R


class Deprojection:
    def __init__(self) -> None:
        self.count = 0 
        rospy.sleep(0.2)

        # tf buffer : 
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # left camera topics
        left_camera_color_topic = rospy.get_param("~left_color_image_topic","/camera/color/image_raw")
        left_camera_info_topic = rospy.get_param("~left_camera_info_topic","/camera/color/image_raw")
        left_camera_depth_topic = rospy.get_param("~left_depth_image_topic","/camera/color/image_raw")
        left_detected_image_topic = f"/left_camera/hand_detected_image_raw"
        
        # right camera topics
        right_camera_color_topic = rospy.get_param("~right_color_image_topic","/camera/color/image_raw")
        right_camera_info_topic = rospy.get_param("~right_camera_info_topic","/camera/color/image_raw")
        right_camera_depth_topic = rospy.get_param("~right_depth_image_topic","/camera/color/image_raw")
        right_detected_image_topic = f"/right_camera/hand_detected_image_raw"

        # left camera elements
        self.left_depth_image = None
        self.left_camera_info = None
        self.left_grayscale_image = None
        self.left_color_image = None
        self.left_camera_model = PinholeCameraModel()
        self.left_detected_image = None
        self.base_to_left_cam_tf_mat = np.eye(4,4)
        
        # right camera elements
        self.right_depth_image = None
        self.right_camera_info = None
        self.right_grayscale_image = None
        self.right_color_image = None
        self.right_camera_model = PinholeCameraModel()
        self.right_detected_image = None
        self.base_to_right_cam_tf_mat = np.eye(4,4)

        # left detection elements
        self.left_rvec = None
        self.left_tvec = None
        self.left_tvec_impulse = None
        self.left_filtered_tvec = np.zeros(shape=(3,),dtype=float)
        self.left_qvec = None
        self.left_qvec_impulse = None
        self.left_filtered_qvec = np.array([0,0,0,1],dtype=float)
        self.left_confidence_threshold = rospy.get_param("~left_confidence_threshold",0.5)
        
        # right detection elements
        self.right_rvec = None
        self.right_tvec = None
        self.right_tvec_impulse = None
        self.right_filtered_tvec = np.zeros(shape=(3,),dtype=float)
        self.right_qvec = None
        self.right_qvec_impulse = None
        self.right_filtered_qvec = np.array([0,0,0,1],dtype=float)
        self.right_confidence_threshold = rospy.get_param("~right_confidence_threshold",0.5)

        # object topics
        self.tvec = None
        self.qvec = None
        self.filtered_tvec = np.array([0,0,0],dtype = float)
        self.filtered_qvec = np.array([0,0,0,1],dtype = float)

        # left linear IIR filter elements
        self.left_alpha = rospy.get_param("~left_alpha",0.25)
        
        # right linear IIR filter elements
        self.right_alpha = rospy.get_param("~right_alpha",0.25)

        # overall IIR filter element 
        self.alpha = rospy.get_param("~alpha",0.25)

        # hand_side 
        self.hand_side = rospy.get_param("~hand_side","Right")


        # left mp initialization
        self.left_mp_hands = mp.solutions.hands
        self.left_mp_drawing = mp.solutions.drawing_utils
        self.left_hands =  self.left_mp_hands.Hands(static_image_mode=False,max_num_hands=5,min_detection_confidence=self.left_confidence_threshold)
        
        # right mp initialization
        self.right_mp_hands = mp.solutions.hands
        self.right_mp_drawing = mp.solutions.drawing_utils
        self.right_hands =  self.right_mp_hands.Hands(static_image_mode=False,max_num_hands=5,min_detection_confidence=self.right_confidence_threshold)

        # bounding box in base link frame for detecting false positives
        self.bounding_box_world_x_min = rospy.get_param("~bounding_box/x_min",0.0)
        self.bounding_box_world_x_max = rospy.get_param("~bounding_box/x_max",2.0)
        self.bounding_box_world_y_min = rospy.get_param("~bounding_box/y_min",0.0)
        self.bounding_box_world_y_max = rospy.get_param("~bounding_box/y_max",2.0)
        self.bounding_box_world_z_min = rospy.get_param("~bounding_box/z_min",0.0)
        self.bounding_box_world_z_max = rospy.get_param("~bounding_box/z_max",2.0)

        self.cv_bridge = cv_bridge.CvBridge()
        self.detection_rate = 20
        
        self.mutex = threading.Lock()
        
        self.object_name = rospy.get_param("~object/name","object0")    
        rospy.loginfo("Mediapipe ros node started with params : ")
        rospy.loginfo("Object Name : %s"%self.object_name)
        rospy.loginfo("Left Linear IIR Alpha : %s"%self.left_alpha)
        rospy.loginfo("Left Color image topic : %s"%left_camera_color_topic)
        rospy.loginfo("Left Camera info topic : %s"%left_camera_info_topic)
        rospy.loginfo("Left Depth image topic : %s"%left_camera_depth_topic)

        # Pose topics
        left_pose_topic = f"/left/{self.object_name}_pose"
        left_filtered_pose_topic = f"/left/{self.object_name}_filtered_pose"
        right_pose_topic = f"/right/{self.object_name}_pose"
        right_filtered_pose_topic = f"/right/{self.object_name}_filtered_pose"
        object_pose_topic = f"{self.object_name}_pose"
        object_filtered_pose_topic = f"{self.object_name}_filtered_pose"

        # Subscribers
        self.left_depth_image_sub = rospy.Subscriber(left_camera_depth_topic, Image, self.left_depth_image_callback)
        self.left_camera_info_sub = rospy.Subscriber(left_camera_info_topic, CameraInfo, self.left_camera_info_callback)
        self.left_color_image_sub = rospy.Subscriber(left_camera_color_topic, Image, self.left_color_image_callback)
        self.right_depth_image_sub = rospy.Subscriber(right_camera_depth_topic, Image, self.right_depth_image_callback)
        self.right_camera_info_sub = rospy.Subscriber(right_camera_info_topic, CameraInfo, self.right_camera_info_callback)
        self.right_color_image_sub = rospy.Subscriber(right_camera_color_topic, Image, self.right_color_image_callback)

        # Publishers
        self.left_detected_image_pub = rospy.Publisher(left_detected_image_topic, Image, queue_size=10)
        self.right_detected_image_pub = rospy.Publisher(right_detected_image_topic, Image, queue_size=10)

        self.left_pose_pub = rospy.Publisher(left_pose_topic,PoseStamped,queue_size=10)
        self.left_filtered_pose_pub = rospy.Publisher(left_filtered_pose_topic,PoseStamped,queue_size=10)
        self.right_pose_pub = rospy.Publisher(right_pose_topic,PoseStamped,queue_size=10)
        self.right_filtered_pose_pub = rospy.Publisher(right_filtered_pose_topic,PoseStamped,queue_size=10)

        self.object_pose_pub = rospy.Publisher(object_pose_topic,PoseStamped,queue_size=10)
        self.object_filtered_pose_pub = rospy.Publisher(object_filtered_pose_topic,PoseStamped,queue_size=10)

        # Timers
        self.left_hand_detector_timer = rospy.Timer(rospy.Duration(1/self.detection_rate),callback=self.left_hand_detector_cb)
        self.left_detected_image_publisher_timer = rospy.Timer(rospy.Duration(1/self.detection_rate),callback=self.left_detected_image_publisher_cb)
        self.left_filtered_pose_publisher_timer = rospy.Timer(rospy.Duration(1/self.detection_rate),callback=self.left_filtered_pose_publisher_cb)
        self.left_pose_publisher_timer = rospy.Timer(rospy.Duration(1/self.detection_rate),callback=self.left_pose_publisher_cb)

        self.right_hand_detector_timer = rospy.Timer(rospy.Duration(1/self.detection_rate),callback=self.right_hand_detector_cb)
        self.right_detected_image_publisher_timer = rospy.Timer(rospy.Duration(1/self.detection_rate),callback=self.right_detected_image_publisher_cb)
        self.right_filtered_pose_publisher_timer = rospy.Timer(rospy.Duration(1/self.detection_rate),callback=self.right_filtered_pose_publisher_cb)
        self.right_pose_publisher_timer = rospy.Timer(rospy.Duration(1/self.detection_rate),callback=self.right_pose_publisher_cb)
        
        self.calculate_pose_timer = rospy.Timer(rospy.Duration(1/self.detection_rate),callback=self.calculate_pose)
        self.filter_pose_timer = rospy.Timer(rospy.Duration(1/self.detection_rate),callback=self.filter_pose)
        self.pose_publisher_timer = rospy.Timer(rospy.Duration(1/self.detection_rate),callback=self.pose_publisher_cb)

    def __del__(self):
        del self.left_camera_model
        del self.right_camera_model

    def filter_pose(self,event):
        if self.tvec is None : 
            return
        self.filtered_tvec = self.alpha*self.filtered_tvec + (1-self.alpha)*self.tvec

        if self.qvec is None :
            return
        q1 = self.filtered_qvec
        q2 = self.qvec

        dot = np.dot(q1,q2)
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        if dot > 0.9995:
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
        self.filtered_qvec = filtered_quat

    def pose_publisher_cb(self,event):
        if self.qvec is None or self.tvec is None : 
            return

        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = self.tvec[0]
        msg.pose.position.y = self.tvec[1]
        msg.pose.position.z = self.tvec[2]
        msg.pose.orientation.x = self.qvec[0]
        msg.pose.orientation.y = self.qvec[1]
        msg.pose.orientation.z = self.qvec[2]
        msg.pose.orientation.w = self.qvec[3]

        self.object_pose_pub.publish(msg)

        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = self.filtered_tvec[0]
        msg.pose.position.y = self.filtered_tvec[1]
        msg.pose.position.z = self.filtered_tvec[2]
        msg.pose.orientation.x = self.filtered_qvec[0]
        msg.pose.orientation.y = self.filtered_qvec[1]
        msg.pose.orientation.z = self.filtered_qvec[2]
        msg.pose.orientation.w = self.filtered_qvec[3]

        self.object_filtered_pose_pub.publish(msg)

    def calculate_pose(self,event):
        # here we need to update self.tvec and self.qvec by determining the self.left_tvec, self.left_qvec in world frame, self.right_tvec and self.right_qvec in world frame, averaging
        left_world_tvec = None 
        right_world_tvec = None 

        if self.left_tvec is not None and self.left_camera_info is not None:
            left_point = tf2_geometry_msgs.PointStamped()
            left_point.header.frame_id = self.left_camera_info.header.frame_id
            left_point.header.stamp = rospy.Time.now()
            left_point.point.x = self.left_tvec[0]
            left_point.point.y = self.left_tvec[1]
            left_point.point.z = self.left_tvec[2]
            left_world_point = self.transform_point(left_point,"world")
            left_world_tvec = np.array([left_world_point.point.x,left_world_point.point.y,left_world_point.point.z],dtype=float)
        
        if self.right_tvec is not None and self.right_camera_info is not None:
            right_point = tf2_geometry_msgs.PointStamped()
            right_point.header.frame_id = self.right_camera_info.header.frame_id
            right_point.header.stamp = rospy.Time.now()
            right_point.point.x = self.right_tvec[0]
            right_point.point.y = self.right_tvec[1]
            right_point.point.z = self.right_tvec[2]
            right_world_point = self.transform_point(right_point,"world")
            right_world_tvec = np.array([right_world_point.point.x,right_world_point.point.y,right_world_point.point.z],dtype=float)

        if left_world_tvec is None and right_world_tvec is None :
            pass
        elif left_world_tvec is not None and right_world_tvec is None :
            self.tvec = left_world_tvec
        elif left_world_tvec is None and right_world_tvec is not None :
            self.tvec = right_world_tvec
        elif left_world_tvec is not None and right_world_tvec is not None :
            self.tvec = (left_world_tvec + right_world_tvec)/2

        self.qvec = np.array([0,0,0,1],dtype=float)
        print(self.tvec)
        print(self.qvec)
    
    def get_average_pixel_xy_per_hand(self,results, image_shape):
        height, width, _ = image_shape
        hand_data = []

        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                total_x = 0
                total_y = 0
                count = 0

                for lm in hand_landmarks.landmark:
                    pixel_x = lm.x * width
                    pixel_y = lm.y * height
                    total_x += pixel_x
                    total_y += pixel_y
                    count += 1

                if count > 0:
                    avg_x = int(total_x / count)
                    avg_y = int(total_y / count)
                    classification = handedness.classification[0]
                    hand_data.append({
                        "x": avg_x, # this is the columns
                        "y": avg_y, # this is the rows
                        "conf": round(classification.score, 2),
                        "label": classification.label  # "Left" or "Right"
                    })

        return hand_data

    def print_hand_landmarks_from_results(self,results):
        if results.multi_hand_landmarks and results.multi_handedness:
            for idx, (hand_landmarks, handedness) in enumerate(zip(results.multi_hand_landmarks, results.multi_handedness)):
                hand_type = handedness.classification[0].label  # 'Left' or 'Right'
                confidence = handedness.classification[0].score
                print(f"\nHand {idx + 1} - Type: {hand_type} (Confidence: {confidence:.2f})")
                for i, lm in enumerate(hand_landmarks.landmark):
                    print(f"  Landmark {i:2d}: x={lm.x:.3f}, y={lm.y:.3f}, z={lm.z:.3f}")
        else:
            print("No hands detected.")
  
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
    
    def transform_pose(self, pose, target_frame):
        try:
            return self.tf_buffer.transform(pose, target_frame, rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transform failed: {e}")
            return None

    def transform_point(self, point, target_frame):
        try:
            return self.tf_buffer.transform(point, target_frame, rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transform failed: {e}")
            return None

    def get_hand_position_3d_left_cam(self,hand_data):
        # need to detect all hand coordinates, select the ones in the bounding box and then sort by confidence, and then select the coordinate closes to the object
        position = None
        for hand in hand_data:
            u, v = int(hand["x"]), int(hand["y"])  # pixel coordinates (x = col, y = row)
            ray = self.left_camera_model.projectPixelTo3dRay((u, v)) 
            depth_image = self.left_depth_image.copy()
            mask = np.uint8(depth_image == 0) * 255 
            # mask = np.uint8(mask == None) * 255 
            depth_image_filtered = cv2.inpaint(depth_image, mask, inpaintRadius=3, flags=cv2.INPAINT_NS)
            if v < self.left_camera_info.height and u < self.left_camera_info.width:
                z = depth_image_filtered[v, u] / 1000.0 
            else :
                return None
            x = ray[0] * z
            y = ray[1] * z
            z = ray[2] * z
            
            # need to validate here
            pose = tf2_geometry_msgs.PoseStamped()
            pose.header.frame_id = self.left_camera_info.header.frame_id
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1

            pose_wrt_world = self.transform_pose(pose,"world")

            case_1 = self.bounding_box_world_x_min <= pose_wrt_world.pose.position.x <= self.bounding_box_world_x_max
            case_2 = self.bounding_box_world_y_min <= pose_wrt_world.pose.position.y <= self.bounding_box_world_y_max
            case_3 = self.bounding_box_world_z_min <= pose_wrt_world.pose.position.z <= self.bounding_box_world_z_max

            if case_1 and case_2 and case_3 :
                position = [x,y,z]
                break
            else:
                position = None
        return position

    def left_hand_detector_cb(self,event):
        if self.left_color_image is None : 
            return
        image_detected,rvec,tvec = self.left_detect_hands(self.left_color_image)
        self.left_detected_image = image_detected
        
        if tvec is not None and rvec is not None:
            self.left_tvec = tvec
            r = R.from_rotvec(rvec)
            self.left_qvec = r.as_quat()
        else :
            self.left_tvec = None
            self.left_qvec = None

    def left_detect_hands(self,image):
        # Convert BGR to RGB
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.left_hands.process(image_rgb)
        rvec = None
        tvec = None

        # Draw the hand annotations on the image
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.left_mp_drawing.draw_landmarks(
                    image,  # draw on original BGR image
                    hand_landmarks,
                    self.left_mp_hands.HAND_CONNECTIONS,
                    self.left_mp_drawing.DrawingSpec(color=(0,255,0), thickness=2, circle_radius=2),
                    self.left_mp_drawing.DrawingSpec(color=(255,0,0), thickness=2))

        if results.multi_hand_landmarks is not None and not self.count : # this is the condition to check if there are hands exist or not
            hand_data = self.get_average_pixel_xy_per_hand(results,image.shape)
            for hand in hand_data:
                cv2.circle(image,(hand["x"],hand["y"]),5,(0,0,255),5)
            
            position = self.get_hand_position_3d_left_cam(hand_data)

            if self.left_camera_model is not None and position is not None: 
                intrinsics = self.left_camera_model.K
                distortion_coefficients = self.left_camera_model.D
                unit_rotation_mat = np.eye(3,3,dtype=float)
                rvec = np.zeros((3,),dtype=float)
                rvec, _ = cv2.Rodrigues(unit_rotation_mat)
                tvec = np.array(position,dtype = float)
                image = self.draw_axes(image,rvec,tvec,intrinsics,distortion_coefficients)
            else : 
                return image,None,None
        else : 
            return image,None,None
        
        return image,rvec.reshape((3,)),tvec

    def left_detected_image_publisher_cb(self,event):
        if self.left_detected_image is not None:
            image = self.cv_bridge.cv2_to_imgmsg(cvim=self.left_detected_image,encoding="bgr8")
            self.left_detected_image_pub.publish(image)

    def left_pose_publisher_cb(self,event):
        if self.left_qvec is None or self.left_tvec is None : 
            return

        msg = PoseStamped()
        msg.header.frame_id = self.left_camera_info.header.frame_id
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = self.left_tvec[0]
        msg.pose.position.y = self.left_tvec[1]
        msg.pose.position.z = self.left_tvec[2]
        msg.pose.orientation.x = self.left_qvec[0]
        msg.pose.orientation.y = self.left_qvec[1]
        msg.pose.orientation.z = self.left_qvec[2]
        msg.pose.orientation.w = self.left_qvec[3]

        self.left_pose_pub.publish(msg)        

    def left_filter_pose(self):
        # filter self.tvec and self.qvec with some alpha and update self.filtered_tvec and self.filtered_qvec
        q1 = self.left_filtered_qvec
        q2 = self.left_qvec

        dot = np.dot(q1,q2)
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        if dot > 0.9995:
            # Linear interpolation
            filtered_quat = (self.left_alpha)*q1 + (1-self.left_alpha)*q2
        else : 
            theta_0 = np.arccos(dot)
            sin_theta_0 = np.sin(theta_0)
            theta = theta_0*(1-self.left_alpha)
            sin_theta = np.sin(theta)

            s1 = np.sin(theta_0 - theta) / sin_theta_0
            s2 = sin_theta / sin_theta_0

            filtered_quat = s1 * q1 + s2 * q2
        filtered_quat = filtered_quat / np.linalg.norm(filtered_quat)
        #SLERP for qvec
        self.left_filtered_qvec = filtered_quat
        #LERP for tvec
        self.left_filtered_tvec = self.left_alpha*self.left_filtered_tvec + (1-self.left_alpha)*self.left_tvec

    def left_filtered_pose_publisher_cb(self,event):
        if self.left_tvec is None or self.left_qvec is None :
            return
        
        self.left_filter_pose()
        msg = PoseStamped()
        msg.header.frame_id = self.left_camera_info.header.frame_id
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = self.left_filtered_tvec[0]
        msg.pose.position.y = self.left_filtered_tvec[1]
        msg.pose.position.z = self.left_filtered_tvec[2]
        msg.pose.orientation.x = self.left_filtered_qvec[0]
        msg.pose.orientation.y = self.left_filtered_qvec[1]
        msg.pose.orientation.z = self.left_filtered_qvec[2]
        msg.pose.orientation.w = self.left_filtered_qvec[3]
        self.left_filtered_pose_pub.publish(msg)
        
    def left_color_image_callback(self, msg: Image):
        self.left_grayscale_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        self.left_color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def left_depth_image_callback(self, msg: Image):
        self.left_depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def left_camera_info_callback(self, msg: CameraInfo):
        self.left_camera_info = msg
        self.left_camera_model.fromCameraInfo(msg)

    ##############################################################

    def get_hand_position_3d_right_cam(self,hand_data):
        # need to detect all hand coordinates, select the ones in the bounding box and then sort by confidence, and then select the coordinate closes to the object
        position = None
        for hand in hand_data:
            u, v = int(hand["x"]), int(hand["y"])  # pixel coordinates (x = col, y = row)
            ray = self.right_camera_model.projectPixelTo3dRay((u, v)) 
            depth_image = self.right_depth_image.copy()
            mask = np.uint8(depth_image == 0) * 255 
            # mask = np.uint8(mask == None) * 255 
            depth_image_filtered = cv2.inpaint(depth_image, mask, inpaintRadius=3, flags=cv2.INPAINT_NS)
            z = depth_image_filtered[v, u] / 1000.0 
            x = ray[0] * z
            y = ray[1] * z
            z = ray[2] * z

            # need to validate here
            pose = tf2_geometry_msgs.PoseStamped()
            pose.header.frame_id = self.right_camera_info.header.frame_id
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1

            pose_wrt_world = self.transform_pose(pose,"world")

            case_1 = self.bounding_box_world_x_min <= pose_wrt_world.pose.position.x <= self.bounding_box_world_x_max
            case_2 = self.bounding_box_world_y_min <= pose_wrt_world.pose.position.y <= self.bounding_box_world_y_max
            case_3 = self.bounding_box_world_z_min <= pose_wrt_world.pose.position.z <= self.bounding_box_world_z_max

            if case_1 and case_2 and case_3 :
                position = [x,y,z]
                break
            else:
                position = None

        return position
            
    def right_hand_detector_cb(self,event):
        if self.right_color_image is None : 
            return
        image_detected,rvec,tvec = self.right_detect_hands(self.right_color_image)
        self.right_detected_image = image_detected
        
        if tvec is not None and rvec is not None:
            self.right_tvec = tvec
            r = R.from_rotvec(rvec)
            self.right_qvec = r.as_quat()
        else :
            self.right_tvec = None
            self.right_qvec = None

    def right_detect_hands(self,image):
        # Convert BGR to RGB
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.right_hands.process(image_rgb)
        rvec = None
        tvec = None

        # Draw the hand annotations on the image
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.right_mp_drawing.draw_landmarks(
                    image,  # draw on original BGR image
                    hand_landmarks,
                    self.right_mp_hands.HAND_CONNECTIONS,
                    self.right_mp_drawing.DrawingSpec(color=(0,255,0), thickness=2, circle_radius=2),
                    self.right_mp_drawing.DrawingSpec(color=(255,0,0), thickness=2))

        if results.multi_hand_landmarks is not None and not self.count : # this is the condition to check if there are hands exist or not
            hand_data = self.get_average_pixel_xy_per_hand(results,image.shape)
            for hand in hand_data:
                cv2.circle(image,(hand["x"],hand["y"]),5,(0,0,255),5)
            
            position = self.get_hand_position_3d_right_cam(hand_data)

            if self.right_camera_model is not None and position is not None: 
                intrinsics = self.right_camera_model.K
                distortion_coefficients = self.right_camera_model.D
                unit_rotation_mat = np.eye(3,3,dtype=float)
                rvec = np.zeros((3,),dtype=float)
                rvec, _ = cv2.Rodrigues(unit_rotation_mat)
                tvec = np.array(position,dtype = float)
                image = self.draw_axes(image,rvec,tvec,intrinsics,distortion_coefficients)
            else : 
                return image,None,None
        else : 
            return image,None,None
        
        return image,rvec.reshape((3,)),tvec
    
    def right_detected_image_publisher_cb(self,event):
        if self.right_detected_image is not None:
            image = self.cv_bridge.cv2_to_imgmsg(cvim=self.right_detected_image,encoding="bgr8")
            self.right_detected_image_pub.publish(image)

    def right_pose_publisher_cb(self,event):
        if self.right_qvec is None or self.right_tvec is None : 
            return

        msg = PoseStamped()
        msg.header.frame_id = self.right_camera_info.header.frame_id
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = self.right_tvec[0]
        msg.pose.position.y = self.right_tvec[1]
        msg.pose.position.z = self.right_tvec[2]
        msg.pose.orientation.x = self.right_qvec[0]
        msg.pose.orientation.y = self.right_qvec[1]
        msg.pose.orientation.z = self.right_qvec[2]
        msg.pose.orientation.w = self.right_qvec[3]

        self.right_pose_pub.publish(msg)        

    def right_filter_pose(self):
        # filter self.tvec and self.qvec with some alpha and update self.filtered_tvec and self.filtered_qvec
        q1 = self.right_filtered_qvec
        q2 = self.right_qvec

        dot = np.dot(q1,q2)
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        if dot > 0.9995:
            # Linear interpolation
            filtered_quat = (self.right_alpha)*q1 + (1-self.right_alpha)*q2
        else : 
            theta_0 = np.arccos(dot)
            sin_theta_0 = np.sin(theta_0)
            theta = theta_0*(1-self.right_alpha)
            sin_theta = np.sin(theta)

            s1 = np.sin(theta_0 - theta) / sin_theta_0
            s2 = sin_theta / sin_theta_0

            filtered_quat = s1 * q1 + s2 * q2
        filtered_quat = filtered_quat / np.linalg.norm(filtered_quat)
        #SLERP for qvec
        self.right_filtered_qvec = filtered_quat
        #LERP for tvec
        self.right_filtered_tvec = self.right_alpha*self.right_filtered_tvec + (1-self.right_alpha)*self.right_tvec

    def right_filtered_pose_publisher_cb(self,event):
        if self.right_tvec is None or self.right_qvec is None :
            return
        
        self.right_filter_pose()
        msg = PoseStamped()
        msg.header.frame_id = self.right_camera_info.header.frame_id
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = self.right_filtered_tvec[0]
        msg.pose.position.y = self.right_filtered_tvec[1]
        msg.pose.position.z = self.right_filtered_tvec[2]
        msg.pose.orientation.x = self.right_filtered_qvec[0]
        msg.pose.orientation.y = self.right_filtered_qvec[1]
        msg.pose.orientation.z = self.right_filtered_qvec[2]
        msg.pose.orientation.w = self.right_filtered_qvec[3]
        self.right_filtered_pose_pub.publish(msg)
        
    def right_color_image_callback(self, msg: Image):
        self.right_grayscale_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        self.right_color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def right_depth_image_callback(self, msg: Image):
        self.right_depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def right_camera_info_callback(self, msg: CameraInfo):
        self.right_camera_info = msg
        self.right_camera_model.fromCameraInfo(msg)

if __name__ == "__main__":
    rospy.init_node("mediapipe_ros_driver",argv=sys.argv,anonymous=False)
    rospy.sleep(0.15)
    deproject = Deprojection()
    rospy.loginfo("Detector running")
    rospy.spin()