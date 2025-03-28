import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from cv2 import aruco
from geometry_msgs.msg import PoseStamped, PointStamped

class ArucoDetectorNode:
    def __init__(self):
        rospy.init_node("aruco_detector", anonymous=True)
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_size = 0.04  # Marker size in meters

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        self.aruco_params = aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)

        self.image_pub = rospy.Publisher("/aruco_detected_image", Image, queue_size=1)
        self.cube_pose_pub = rospy.Publisher("/cube_pose", PoseStamped, queue_size=1)
        self.edge_positions_pub = rospy.Publisher("/cube_edge_positions", PointStamped, queue_size=8)

        rospy.loginfo("Aruco Detector Node Started")

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            rospy.loginfo("Camera Info Received")

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            rospy.logwarn("Waiting for camera info...")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == 21:
                    aruco.drawDetectedMarkers(cv_image, [corners[i]], np.array([[marker_id]]))

                    success, rvec, tvec = cv2.solvePnP(
                        np.array([[0, 0, 0], [self.marker_size, 0, 0], 
                                  [self.marker_size, self.marker_size, 0], [0, self.marker_size, 0]]),
                        corners[i],
                        self.camera_matrix,
                        self.dist_coeffs
                    )

                    if success:
                        cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

                        t = np.array([0.03, 0.03, 0.03])
                        cube_center = tvec.flatten() + t

                        # Cube corners
                        d = 0.06 / 2  # half of the side length
                        local_corners = np.array([[-d, -d, -d], [d, -d, -d],
                                                  [d, d, -d], [-d, d, -d],
                                                  [-d, -d, d], [d, -d, d],
                                                  [d, d, d], [-d, d, d]])

                        # extrinsic projection
                        world_corners = [cube_center + corner for corner in local_corners]

                        # publish the edges
                        for corner in world_corners:
                            edge_pos = PointStamped()
                            edge_pos.header.stamp = rospy.Time.now()
                            edge_pos.header.frame_id = "camera_color_optical_frame"
                            edge_pos.point.x = corner[0]
                            edge_pos.point.y = corner[1]
                            edge_pos.point.z = corner[2]
                            self.edge_positions_pub.publish(edge_pos)

                        # Projection
                        projected_corners, _ = cv2.projectPoints(
                            local_corners, rvec, tvec, self.camera_matrix, self.dist_coeffs
                        )
                        projected_corners = projected_corners.reshape(-1, 2).astype(int)

                        for start, end in zip(range(4), [1, 2, 3, 0]): # Draw 3d bbox
                            cv2.line(cv_image, tuple(projected_corners[start]), tuple(projected_corners[end]), (0, 255, 0), 2)
                            cv2.line(cv_image, tuple(projected_corners[start+4]), tuple(projected_corners[end+4]), (0, 255, 0), 2)
                            cv2.line(cv_image, tuple(projected_corners[start]), tuple(projected_corners[start+4]), (0, 255, 0), 2)

                        cube_pose = PoseStamped()
                        cube_pose.header.stamp = rospy.Time.now()
                        cube_pose.header.frame_id = "camera_color_optical_frame"
                        cube_pose.pose.position.x = cube_center[0]
                        cube_pose.pose.position.y = cube_center[1]
                        cube_pose.pose.position.z = cube_center[2]
                        self.cube_pose_pub.publish(cube_pose)

        ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_pub.publish(ros_image)

        cv2.imshow("Aruco Detection", cv_image)
        cv2.waitKey(1)

if __name__ == "__main__":
    try:
        ArucoDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()