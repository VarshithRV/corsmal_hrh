# sample code for realsense type camera
import rospy
from sensor_msgs.msg import CameraInfo, Image
import cv_bridge, cv2
import numpy as np
from image_geometry import PinholeCameraModel
import apriltag
import copy
import threading


class Deprojection:
    def __init__(self) -> None:
        self.depth_image = None
        self.camera_info = None
        self.grayscale_image = None
        self.cv_bridge = cv_bridge.CvBridge()
        self.camera_model = PinholeCameraModel()
        self.apriltag_detector_options = apriltag.DetectorOptions()
        self.apriltag_detector = apriltag.Detector(self.apriltag_detector_options)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict,self.aruco_parameters)
        self.detection_rate = 30
        self.marker_size = 40/1000 # from mm to m
        self.marker_distance = 15/1000 # from mm to m
        self.marker_id = 4
        self.mutex = threading.Lock()
        self.detected_image = None
        
        # Camera topics
        camera_color_topic = f"/camera/color/image_raw"
        camera_info_topic = f"/camera/aligned_depth_to_color/camera_info"
        camera_depth_topic = f"/camera/aligned_depth_to_color/image_raw"
        
        # Subscribers
        self.depth_image_sub = rospy.Subscriber(camera_depth_topic, Image, self.depth_image_callback)
        self.camera_info_sub = rospy.Subscriber(camera_info_topic, CameraInfo, self.camera_info_callback)
        self.color_image_sub = rospy.Subscriber(camera_color_topic, Image, self.color_image_callback)

        # Timers
        # self.apriltag_detector_timer = rospy.Timer(rospy.Duration(1/self.detection_rate),callback=self.apriltag_detector_cb)        
        self.aruco_detector_timer = rospy.Timer(rospy.Duration(1/self.detection_rate),callback=self.aruco_detector_cb)


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
        if self.grayscale_image is None:
            return
        detections = self.apriltag_detector.detect(self.grayscale_image)
        detected_image = cv2.cvtColor(copy.deepcopy(self.grayscale_image),cv2.COLOR_GRAY2RGB)


        if len(detections):
            for i in range(len(detections)):

                x1y1 = detections[i].corners[0] # top left
                x1y2 = detections[i].corners[3] # bottom left
                x2y1 = detections[i].corners[1] # top right
                x2y2 = detections[i].corners[2] # bottom right
                
                detected_image = cv2.circle(detected_image,center=(int(x1y1[0]),int(x1y1[1])),radius=1,color=(0,255,0),thickness=2)
                detected_image = cv2.circle(detected_image,center=(int(x1y2[0]),int(x1y2[1])),radius=1,color=(0,255,0),thickness=2)
                detected_image = cv2.circle(detected_image,center=(int(x2y1[0]),int(x2y1[1])),radius=1,color=(0,255,0),thickness=2)
                detected_image = cv2.circle(detected_image,center=(int(x2y2[0]),int(x2y2[1])),radius=1,color=(0,255,0),thickness=2)
                
                # origin at the bottom left
                if detections[i].tag_id == self.marker_id:
                    object_points = np.array(
                                    [
                                        [0,self.marker_size,0], #top left
                                        [self.marker_size,self.marker_size,0], #top right
                                        [0,0,0], #bottom left
                                        [self.marker_size,0,0] #bottom right
                                     ], dtype=float
                    )

                    image_points = np.stack((x1y1.reshape((1,2)),x2y1.reshape((1,2)),x1y2.reshape((1,2)),x2y2.reshape((1,2))),axis=1)[0]

                    intrinsics = self.camera_model.K
                    distortion_coefficients = self.camera_model.D

                    success, rvec, tvec = cv2.solvePnP(
                        objectPoints=object_points,
                        imagePoints=image_points,
                        cameraMatrix=intrinsics,
                        distCoeffs=distortion_coefficients,
                        flags=cv2.SOLVEPNP_IPPE
                    )

                    if success:
                        detected_image = self.draw_axes(detected_image,rvec,tvec,intrinsics,distortion_coefficients) 
        cv2.imshow("apriltag_detector",detected_image)
        cv2.waitKey(1)

    def aruco_detector_cb(self, event):
        if self.grayscale_image is None:
            return
        corners, ids, _ = self.aruco_detector.detectMarkers(self.grayscale_image)
        detected_image = cv2.cvtColor(copy.deepcopy(self.grayscale_image),cv2.COLOR_GRAY2RGB)
        
        if ids is not None : 
            ids = ids.reshape((ids.shape[0],)).tolist()
            if self.marker_id in ids:
                x1y1 = (corners)[ids.index(self.marker_id)][0][2] # top left
                x1y2 = (corners)[ids.index(self.marker_id)][0][1] # bottom left
                x2y1 = (corners)[ids.index(self.marker_id)][0][3] # top right
                x2y2 = (corners)[ids.index(self.marker_id)][0][0] # bottom right

                detected_image = cv2.circle(detected_image,(int(x1y1[0]),int(x1y1[1])),1,(0,0,255),2)
                detected_image = cv2.circle(detected_image,(int(x1y2[0]),int(x1y2[1])),1,(0,0,255),2)
                detected_image = cv2.circle(detected_image,(int(x2y1[0]),int(x2y1[1])),1,(0,0,255),2)
                detected_image = cv2.circle(detected_image,(int(x2y2[0]),int(x2y2[1])),1,(0,0,255),2)

                # origin at the bottom left
                object_points = np.array(
                                    [
                                        [0,self.marker_size,0], #top left
                                        [self.marker_size,self.marker_size,0], #top right
                                        [0,0,0], #bottom left
                                        [self.marker_size,0,0] #bottom right
                                     ], dtype=float
                    )

                image_points = np.stack((x1y1.reshape((1,2)),x2y1.reshape((1,2)),x1y2.reshape((1,2)),x2y2.reshape((1,2))),axis=1)[0]
                intrinsics = self.camera_model.K
                distortion_coefficients = self.camera_model.D
                success, rvec, tvec = cv2.solvePnP(
                    objectPoints=object_points,
                    imagePoints=image_points,
                    cameraMatrix=intrinsics,
                    distCoeffs=distortion_coefficients,
                    flags=cv2.SOLVEPNP_IPPE
                )
                if success:
                    detected_image = self.draw_axes(detected_image,rvec,tvec,intrinsics,distortion_coefficients)                

        cv2.imshow("aruco_detections",detected_image)
        cv2.waitKey(1)

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
    rospy.init_node("april_detector")
    rospy.sleep(0.15) # some time to initialize stuff
    deproject = Deprojection()
    rospy.loginfo("Detector running")
    rospy.spin()