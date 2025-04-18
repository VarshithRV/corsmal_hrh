import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, PointStamped, Point
import sys
import tf.transformations as t
import yaml, os, copy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion

class pose_estimation():
    def __init__(self):
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        self.cv_bridge = CvBridge()
        self.aruco_pose = None
        self.cuboid_pose = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict,self.aruco_parameters)
        self.annotated_image = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.params = rospy.get_param("/aruco_proxy_pose_estimation")
        self.marker_size = self.params["marker_size"]/1000
        self.LENGTH = self.params["dimension"]["length"]/1000
        self.WIDTH = self.params["dimension"]["width"]/1000
        self.DEPTH = self.params["dimension"]["depth"]/1000
        self.rgb_image_topic = rospy.get_param("~rgb_image_topic","/camera/color/image_raw")
        self.rgb_camera_info_topic = rospy.get_param("~rgb_camera_info_topic","/camera/color/camera_info")
        self.depth_image_topic = rospy.get_param("~depth_image_topic","/camera/aligned_depth_to_color/image_raw")
        self.target_frame = rospy.get_param("~target_frame","base_link")
        self.linear_infinite_impulse_response_coefficient = self.params["filters"]["linear_infinite_impulse_response"]["exponential_smoothing_coefficient"]
        self.filtered_pose = None
        self.rotation_matrix = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.offset_xyz = [0,0,-0.1]
        self.offset_quat = [0,0,0,1]
    
        rospy.Subscriber(self.rgb_camera_info_topic,CameraInfo,callback = self.camera_info_cb)
        rospy.Subscriber(self.rgb_image_topic,Image,callback=self.color_cb)
        rospy.Subscriber(self.depth_image_topic,Image,callback=self.depth_cb)
        rospy.loginfo("%s : waiting for messages now",rospy.get_name())
        rospy.wait_for_message(self.rgb_image_topic,Image)
        rospy.wait_for_message(self.depth_image_topic,Image)
        rospy.wait_for_message(self.rgb_camera_info_topic,CameraInfo)

        self._wait_for_static_rotation()

        self.cuboid_publisher = rospy.Publisher("/cuboid_pose",PoseStamped,queue_size=10)
        self.cuboid_filtered_publisher = rospy.Publisher("/cuboid_pose_filtered",PoseStamped,queue_size=10)

        rospy.Timer(rospy.Duration(1/30),callback=self.publish_aruco_pose) # 30 fps
        rospy.loginfo("%s : Started the rest node with parameters:",rospy.get_name())
        for item in self.params:
            rospy.loginfo(f"{item} : {self.params[item]}")

    def apply_pose_offset(self,original_pose, target_frame, offset_xyz, offset_quat, tf_buffer):
        try:
            # Lookup transform from original_pose's frame to target_frame
            transform = tf_buffer.lookup_transform(
                target_frame,
                original_pose.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0)
            )

            # Transform the pose to the target frame
            pose_in_target = tf2_geometry_msgs.do_transform_pose(original_pose, transform)

            # Create a TransformStamped representing the offset
            offset_transform = TransformStamped()
            offset_transform.header.stamp = rospy.Time.now()
            offset_transform.header.frame_id = target_frame
            offset_transform.child_frame_id = "offset_pose"
            offset_transform.transform.translation.x = offset_xyz[0]
            offset_transform.transform.translation.y = offset_xyz[1]
            offset_transform.transform.translation.z = offset_xyz[2]
            offset_transform.transform.rotation = Quaternion(*offset_quat)

            # Apply the offset transform to the transformed pose
            final_pose = tf2_geometry_msgs.do_transform_pose(pose_in_target, offset_transform)

            return final_pose

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
            rospy.logwarn("Transform failed: %s", str(e))
            return None


    def _wait_for_static_rotation(self):
        rospy.loginfo(f"Waiting for static transform from {self.camera_info.header.frame_id} to {self.target_frame}...")

        while not rospy.is_shutdown() and self.rotation_matrix is None:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    self.camera_info.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(1.0)
                )
                quat = tf_msg.transform.rotation
                self.rotation_matrix = t.quaternion_matrix(
                    [quat.x, quat.y, quat.z, quat.w]
                )[:3, :3]
                rospy.loginfo("Static transform acquired.")
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
                rospy.logerr_throttle(5.0, f"Waiting for static transform: {e}")
                rospy.sleep(1.0)

    def publish_aruco_pose(self,event):
        cuboid_pose = self.detect_aruco()
        if cuboid_pose is None :
            return
        cuboid_pose = self.apply_pose_offset(cuboid_pose,self.target_frame,self.offset_xyz,self.offset_quat,self.tf_buffer)
        cuboid_pose_filtered = None
        if cuboid_pose is not None : 
            cuboid_pose_filtered = self.low_pass_filter_pose(cuboid_pose)
        if cuboid_pose_filtered is not None : 
            self.cuboid_filtered_publisher.publish(cuboid_pose_filtered)

    def average_transformations_batch(self,transformations: np.ndarray) -> np.ndarray:
        assert transformations.ndim == 3 and transformations.shape[1:] == (4, 4), \
            "Input must be of shape (N, 4, 4)"
        N = transformations.shape[0]

        translations = transformations[:, :3, 3]
        avg_translation = np.mean(translations, axis=0)
        
        # Average rotation using SVD
        rotation_matrices = transformations[:, :3, :3]
        R_mean = np.mean(rotation_matrices, axis=0)
        U, _, Vt = np.linalg.svd(R_mean)
        R_avg = U @ Vt
        if np.linalg.det(R_avg) < 0:
            U[:, -1] *= -1
            R_avg = U @ Vt
        T_avg = np.eye(4)
        T_avg[:3, :3] = R_avg
        T_avg[:3, 3] = avg_translation
        return T_avg
        
    def detect_aruco(self):
        gray = cv2.cvtColor(self.color_image,cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        cv_image = self.color_image
        
        if ids is None: 
            return None
        else : 
            # all detections and corners
            aruco_detections = {}
            ids = list(ids.flatten())
            if 21 in ids:
                id_21 = ids.index(21)
                corners_21 = corners[id_21]
                success_21, rvec_21, tvec_21 = cv2.solvePnP(
                                np.array([[0, 0, 0], [self.marker_size, 0, 0], 
                                        [self.marker_size, self.marker_size, 0], [0, self.marker_size, 0]]),
                                corners_21,
                                self.camera_matrix,
                                self.dist_coeffs
                            )
                if success_21 : 
                    rotation_matrix = cv2.Rodrigues(rvec_21)[0]
                    R_21 = np.eye(4)
                    R_21[:3,:3] = rotation_matrix
                    R_21[:3,3] = [0,0,0]
                    R_21[:3, 3] = tvec_21.flatten()
                    transformation_center_cuboid = np.eye(4)
                    transformation_center_cuboid[:4, 3] = [self.marker_size / 2, self.marker_size / 2, self.DEPTH / 2,1]
                    aruco_detections["21"] = {"frame":R_21,"transformation":transformation_center_cuboid}

            if 3 in ids :
                id_3 = ids.index(3)
                corners_3 = corners[id_3]
                success_3, rvec_3, tvec_3 = cv2.solvePnP(
                                np.array([[0, 0, 0], [self.marker_size, 0, 0], 
                                        [self.marker_size, self.marker_size, 0], [0, self.marker_size, 0]]),
                                corners_3,
                                self.camera_matrix,
                                self.dist_coeffs
                            )
                if success_3 : 
                    rotation_matrix = cv2.Rodrigues(rvec_3)[0]
                    R_3 = np.eye(4)
                    R_3[:3,:3] = rotation_matrix
                    R_3[:3,3] = [0,0,0]
                    R_3[:3, 3] = tvec_3.flatten()
                    T = [[1,0,0,self.marker_size/2],[0,0,1,self.marker_size/2],[0,-1,0,self.WIDTH/2],[0,0,0,1]]
                    transformation_center_cuboid = np.array(T)
                    aruco_detections["3"] = {"frame":R_3,"transformation":transformation_center_cuboid}

            if 1 in ids:
                id_1 = ids.index(1)
                corners_1 = corners[id_1]
                success_1, rvec_1, tvec_1 = cv2.solvePnP(
                                np.array([[0, 0, 0], [self.marker_size, 0, 0], 
                                        [self.marker_size, self.marker_size, 0], [0, self.marker_size, 0]]),
                                corners_1,
                                self.camera_matrix,
                                self.dist_coeffs
                            )
                if success_1 : 
                    rotation_matrix = cv2.Rodrigues(rvec_1)[0]
                    R_1 = np.eye(4)
                    R_1[:3,:3] = rotation_matrix
                    R_1[:3,3] = [0,0,0]
                    R_1[:3, 3] = tvec_1.flatten()
                    T = [[0,0,1,self.marker_size/2],[0,1,0,self.marker_size/2],[-1,0,0,self.LENGTH/2],[0,0,0,1]]
                    transformation_center_cuboid = np.array(T)
                    aruco_detections["1"] = {"frame":R_1,"transformation":transformation_center_cuboid}
            
            if 2 in ids : 
                id_2 = ids.index(2)
                corners_2 = corners[id_2]

                success_2, rvec_2, tvec_2 = cv2.solvePnP(
                                np.array([[0, 0, 0], [self.marker_size, 0, 0], 
                                          [self.marker_size, self.marker_size, 0], [0, self.marker_size, 0]]),
                                corners_2,
                                self.camera_matrix,
                                self.dist_coeffs
                            )
                if success_2 : 
                    rotation_matrix = cv2.Rodrigues(rvec_2)[0]
                    R_2 = np.eye(4)
                    R_2[:3,:3] = rotation_matrix
                    R_2[:3,3] = [0,0,0]
                    R_2[:3, 3] = tvec_2.flatten()
                    T = [[1,0,0,self.marker_size/2],[0,0,-1,self.marker_size/2],[0,1,0,self.WIDTH/2],[0,0,0,1]]
                    transformation_center_cuboid = np.array(T)
                    aruco_detections["2"] = {"frame":R_2,"transformation":transformation_center_cuboid}
            if not any(elem in ids for elem in [1,2,3,21]):
                return None

            cuboid_pose = PoseStamped()
            cuboid_pose.header.frame_id = self.camera_info.header.frame_id

            # this has to be computed for all 4 posese
            center_cuboids = []
            for aruco_index in aruco_detections : 
                center_cuboids.append(aruco_detections[aruco_index]["frame"]@aruco_detections[aruco_index]["transformation"])
            
            center_cuboids = np.array(center_cuboids)
            average_center_cuboid = self.average_transformations_batch(center_cuboids)

            center_rvec, _ = cv2.Rodrigues(average_center_cuboid[:3, :3])
            center_tvec = average_center_cuboid[:3, 3].reshape(-1, 1)

            q = t.quaternion_from_matrix(average_center_cuboid)
            cuboid_pose.pose.position.x = center_tvec[0][0]
            cuboid_pose.pose.position.y = center_tvec[1][0]
            cuboid_pose.pose.position.z = center_tvec[2][0]
            cuboid_pose.pose.orientation.x = q[0]
            cuboid_pose.pose.orientation.y = q[1]
            cuboid_pose.pose.orientation.z = q[2]
            cuboid_pose.pose.orientation.w = q[3]

            # the rest is cuboid stuff, that uses the final pose of the cube 
            cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, center_rvec, center_tvec, 0.1)
            corners_wrt_cuboid = np.array([[self.LENGTH/2,self.WIDTH/2,-self.DEPTH/2],[-self.LENGTH/2,self.WIDTH/2,-self.DEPTH/2],
                                          [-self.LENGTH/2,-self.WIDTH/2,-self.DEPTH/2],[self.LENGTH/2,-self.WIDTH/2,-self.DEPTH/2],
                                          [self.LENGTH/2,self.WIDTH/2,self.DEPTH/2],[-self.LENGTH/2,self.WIDTH/2,self.DEPTH/2],
                                          [-self.LENGTH/2,-self.WIDTH/2,self.DEPTH/2],[self.LENGTH/2,-self.WIDTH/2,self.DEPTH/2]])
            
            
            corners_wrt_camera = np.zeros((8, 3))
            i=0
            for cornerns in corners_wrt_cuboid:
                corner_homogeneous = np.append(cornerns, 1)
                corner_transformed = average_center_cuboid@corner_homogeneous
                corners_wrt_camera[i] = corner_transformed[:3]
                i+=1
            corners_wrt_camera_pxpy = np.zeros((8, 2))
            for i, corner in enumerate(corners_wrt_camera):
                corner_2d, _ = cv2.projectPoints(corner.reshape(-1, 3), np.zeros((3, 1)), np.zeros((3, 1)), self.camera_matrix, self.dist_coeffs)
                corners_wrt_camera_pxpy[i] = corner_2d.reshape(-1, 2)
            edges = [[0,1],[1,2],[2,3],[0,3], #front face
                     [4,5],[5,6],[6,7],[4,7],# back face
                     [2,6],[1,5],[0,4],[3,7]# parallel edges
                     ]
            
            for edge in edges:
                pt1 = tuple(corners_wrt_camera_pxpy[edge[0]].astype(int))
                pt2 = tuple(corners_wrt_camera_pxpy[edge[1]].astype(int))
                cv2.line(cv_image, pt1, pt2, (0, 255, 0), 2)
            cv2.imshow("annotated_image",cv_image)
            cv2.waitKey(1)

            
            return cuboid_pose
            
    def low_pass_filter_pose(self, new_pose: PoseStamped) -> PoseStamped:
        alpha = self.linear_infinite_impulse_response_coefficient

        # First-time setup
        if self.filtered_pose is None:
            # Initialize with a deep copy of the first pose
            self.filtered_pose = copy.deepcopy(new_pose)
            return copy.deepcopy(new_pose)

        # Linearly interpolate position
        prev_pos = np.array([
            self.filtered_pose.pose.position.x,
            self.filtered_pose.pose.position.y,
            self.filtered_pose.pose.position.z
        ])
        new_pos = np.array([
            new_pose.pose.position.x,
            new_pose.pose.position.y,
            new_pose.pose.position.z
        ])
        filtered_pos = (1 - alpha) * prev_pos + alpha * new_pos

        # Slerp for orientation
        q1 = [
            self.filtered_pose.pose.orientation.x,
            self.filtered_pose.pose.orientation.y,
            self.filtered_pose.pose.orientation.z,
            self.filtered_pose.pose.orientation.w
        ]
        q2 = [
            new_pose.pose.orientation.x,
            new_pose.pose.orientation.y,
            new_pose.pose.orientation.z,
            new_pose.pose.orientation.w
        ]
        q1 = np.array(q1) / np.linalg.norm(q1)
        q2 = np.array(q2) / np.linalg.norm(q2)

        dot = np.dot(q1, q2)
        if dot < 0.0:
            q2 = -q2
            dot = -dot

        if dot > 0.9995:
            # Linear interpolation for very close quaternions
            filtered_quat = (1 - alpha) * q1 + alpha * q2
        else:
            theta_0 = np.arccos(dot)
            sin_theta_0 = np.sin(theta_0)
            theta = theta_0 * alpha
            sin_theta = np.sin(theta)

            s1 = np.sin(theta_0 - theta) / sin_theta_0
            s2 = sin_theta / sin_theta_0

            filtered_quat = s1 * q1 + s2 * q2

        filtered_quat = filtered_quat / np.linalg.norm(filtered_quat)

        # Update and return filtered pose
        filtered_pose = PoseStamped()
        filtered_pose.header = new_pose.header
        filtered_pose.pose.position.x = filtered_pos[0]
        filtered_pose.pose.position.y = filtered_pos[1]
        filtered_pose.pose.position.z = filtered_pos[2]
        filtered_pose.pose.orientation.x = filtered_quat[0]
        filtered_pose.pose.orientation.y = filtered_quat[1]
        filtered_pose.pose.orientation.z = filtered_quat[2]
        filtered_pose.pose.orientation.w = filtered_quat[3]

        self.filtered_pose = filtered_pose
        return filtered_pose

    def depth_cb(self,msg:Image):
        ros_image = msg
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(ros_image,"passthrough")

    def color_cb(self,msg:Image):
        ros_image = msg
        self.color_image = self.cv_bridge.imgmsg_to_cv2(ros_image,"bgr8")

    def camera_info_cb(self,msg:CameraInfo):
        self.camera_info = msg
        self.camera_matrix = np.array(msg.K).reshape([3,3])
        self.dist_coeffs = np.array(msg.D)



if __name__ == "__main__":
    rospy.init_node("proxy_object_pose_estimator",argv=sys.argv,anonymous=False)
    estimator = pose_estimation()
    rospy.sleep(0.01)
    rospy.spin()
