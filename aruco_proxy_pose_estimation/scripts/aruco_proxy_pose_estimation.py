import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, PointStamped, Point
import sys
import tf.transformations as t

LENGTH = 0.115
WIDTH = 0.06
DEPTH = 0.08

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
        self.marker_size = 0.04
        self.camera_matrix = None
        self.dist_coeffs = None
    
        rospy.Subscriber("/camera/color/camera_info",CameraInfo,callback = self.camera_info_cb)
        rospy.Subscriber("/camera/color/image_raw",Image,callback=self.color_cb)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw",Image,callback=self.depth_cb)

        rospy.wait_for_message("/camera/color/image_raw",Image)
        rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw",Image)
        rospy.wait_for_message("/camera/color/camera_info",CameraInfo)

        self.cuboid_publisher = rospy.Publisher("/cuboid_pose",PoseStamped,queue_size=10)

        rospy.Timer(rospy.Duration(1/30),callback=self.publish_aruco_pose) # 30 fps


    def publish_aruco_pose(self,event):
        cuboid_pose = self.detect_aruco()
        if cuboid_pose is not None : 
            self.cuboid_publisher.publish(cuboid_pose)


    def detect_aruco(self):
        gray = cv2.cvtColor(self.color_image,cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        if ids is None: 
            return None
        else : 
            id_21 = list(ids.flatten()).index(21)
            corners_21 = corners[id_21]
            center_pxpy = corners[id_21].mean(axis=1)
            cv_image = self.color_image

            # find the tvec and rvec of the marker
            success, rvec, tvec = cv2.solvePnP(
                            np.array([[0, 0, 0], [self.marker_size, 0, 0], 
                                      [self.marker_size, self.marker_size, 0], [0, self.marker_size, 0]]),
                            corners_21,
                            self.camera_matrix,
                            self.dist_coeffs
                        )
            
            
            if success:
                aruco_pose = PoseStamped()
                cuboid_pose = PoseStamped()
                aruco_pose.header.frame_id = "camera_color_optical_frame"
                cuboid_pose.header.frame_id = "camera_color_optical_frame"
                rotation_matrix = cv2.Rodrigues(rvec)[0]
                R = np.eye(4)
                R[:3,:3] = rotation_matrix
                R[:3,3] = [0,0,0]
                R[:3, 3] = tvec.flatten()

                # R is the transformation matrix of the aruco
                q = t.quaternion_from_matrix(R)

                aruco_pose.pose.position.x = tvec[0][0]
                aruco_pose.pose.position.y = tvec[1][0]
                aruco_pose.pose.position.z = tvec[2][0]
                aruco_pose.pose.orientation.x = q[0]
                aruco_pose.pose.orientation.y = q[1]
                aruco_pose.pose.orientation.z = q[2]
                aruco_pose.pose.orientation.w = q[3]

                transformation_center_cuboid = np.eye(4)
                transformation_center_cuboid[:4, 3] = [self.marker_size / 2, self.marker_size / 2, DEPTH / 2,1]

                # transformation matrix of the center of the cuboid
                # center_cuboid = np.dot(R,transformation_center_cuboid)
                center_cuboid = R@transformation_center_cuboid
                center_rvec, _ = cv2.Rodrigues(center_cuboid[:3, :3])
                center_tvec = center_cuboid[:3, 3].reshape(-1, 1)
                # R is the transformation matrix of the aruco
                q = t.quaternion_from_matrix(center_cuboid)
                cuboid_pose.pose.position.x = center_tvec[0][0]
                cuboid_pose.pose.position.y = center_tvec[1][0]
                cuboid_pose.pose.position.z = center_tvec[2][0]
                cuboid_pose.pose.orientation.x = q[0]
                cuboid_pose.pose.orientation.y = q[1]
                cuboid_pose.pose.orientation.z = q[2]
                cuboid_pose.pose.orientation.w = q[3]
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, center_rvec, center_tvec, 0.1)

                
                corners_wrt_cuboid = np.array([[LENGTH/2,WIDTH/2,-DEPTH/2],[-LENGTH/2,WIDTH/2,-DEPTH/2],
                                              [-LENGTH/2,-WIDTH/2,-DEPTH/2],[LENGTH/2,-WIDTH/2,-DEPTH/2],
                                              [LENGTH/2,WIDTH/2,DEPTH/2],[-LENGTH/2,WIDTH/2,DEPTH/2],
                                              [-LENGTH/2,-WIDTH/2,DEPTH/2],[LENGTH/2,-WIDTH/2,DEPTH/2]])
                
                
                corners_wrt_camera = np.zeros((8, 3))
                i=0
                for cornerns in corners_wrt_cuboid:
                    corner_homogeneous = np.append(cornerns, 1)
                    corner_transformed = center_cuboid@corner_homogeneous
                    corners_wrt_camera[i] = corner_transformed[:3]
                    i+=1

                corners_wrt_camera_pxpy = np.zeros((8, 2))
                for i, corner in enumerate(corners_wrt_camera):
                    corner_2d, _ = cv2.projectPoints(corner.reshape(-1, 3), np.zeros((3, 1)), np.zeros((3, 1)), self.camera_matrix, self.dist_coeffs)
                    corners_wrt_camera_pxpy[i] = corner_2d.reshape(-1, 2)

                cv2.aruco.drawDetectedMarkers(cv_image, [corners_21], np.array([21]))
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
            
            return None

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
