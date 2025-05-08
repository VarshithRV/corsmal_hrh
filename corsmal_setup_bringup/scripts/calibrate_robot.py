# this script should subscribe to all the posestampeds detected by the cameras and calibrate the left, right camera and the robot

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
import sys
from scipy.spatial.transform import Rotation
import tf.transformations as tft, tf
import numpy as np
import json
import os
from tf2_ros import StaticTransformBroadcaster

# change this to the actual target extrinsic and calibration filenames
left_camera_calibration_target_file_name = "left_cam_test.json"
right_camera_calibration_target_file_name = "right_cam_test.json"
robot_calibration_target_file_name = "robot_test.json"

ccofgp = None
lcofgp = None
rcofgp = None

def ccofgp_cb(msg):
    global ccofgp
    ccofgp = msg

def lcofgp_cb(msg):
    global lcofgp
    lcofgp = msg

def rcofgp_cb(msg):
    global rcofgp
    rcofgp = msg

def broadcast_static_transform(child_frame,parent_frame,translation,rotation):
    static_transform_broadcaster = StaticTransformBroadcaster()
    static_transform = TransformStamped()
    static_transform.header.stamp = rospy.Time.now()
    static_transform.child_frame_id = child_frame
    static_transform.header.frame_id = parent_frame
    static_transform.transform.translation.x = translation[0]
    static_transform.transform.translation.y = translation[1]
    static_transform.transform.translation.z = translation[2]
    static_transform.transform.rotation.x = rotation[0]
    static_transform.transform.rotation.y = rotation[1]
    static_transform.transform.rotation.z = rotation[2]
    static_transform.transform.rotation.w = rotation[3]
    static_transform_broadcaster.sendTransform(static_transform)
    rospy.loginfo(f"Published static transform from {parent_frame} to {child_frame}")

def get_transformation(frame_a, frame_b):
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            # Wait for tf transform from frame_a to frame_b
            (trans, rot) = listener.lookupTransform(frame_a, frame_b, rospy.Time(0))
            # Output translation and rotation
            rospy.loginfo(f"Translation: {trans}")
            rospy.loginfo(f"Rotation (quaternion): {rot}")
            # Convert quaternion to Euler angles for better understanding
            euler = tft.euler_from_quaternion(rot)
            rospy.loginfo(f"Rotation (euler): {euler}")
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            pass
        rate.sleep()

def calibrate_robot():
    global ccofgp
    if ccofgp is None : 
        rospy.loginfo("collocated camera detect object is none")
    translation = np.array([ccofgp.pose.position.x,ccofgp.pose.position.y,ccofgp.pose.position.z],dtype=float)
    rotation_q = np.array([ccofgp.pose.orientation.x,ccofgp.pose.orientation.y,ccofgp.pose.orientation.z,ccofgp.pose.orientation.w],dtype=float)
    R = Rotation.from_quat(rotation_q)
    T = np.eye(4,4)
    T[:3,:3] = R.as_matrix()
    T[:3,3] = translation
    T_inv = np.linalg.inv(T) # we have world to camera color optical here

    camera_translation, camera_rotation_quat = get_transformation("collocated_camera_color_optical_frame","base_link")
    R_cam = Rotation.from_quat(camera_rotation_quat)
    T_robot = np.eye(4,4)
    T_robot[:3,:3] = R_cam.as_matrix()
    T_robot[:3,3] = camera_translation

    T_inv = T_inv@T_robot

    translation_inv = T_inv[:3,3]
    R_inv = Rotation.from_matrix(T_inv[:3,:3])
    rotation_inv_quat = R_inv.as_quat()

    # Save the transformation to a file in the ../config/ directory
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'../config/{robot_calibration_target_file_name}')
    transformation = {"translation": translation_inv.tolist(), "rotation": rotation_inv_quat.tolist()}
    rospy.loginfo(f"Transformation from world to base_link found is {transformation}")
    transformation_json = json.dumps(transformation)
    with open(config_path, 'w') as file:
            file.write(transformation_json)

    rospy.loginfo(f"Transformation calibration of robot successfull, written into file config/{robot_calibration_target_file_name}")
    broadcast_static_transform("base_link","world",translation_inv,rotation_inv_quat)
    return

def calibrate_left_camera():
    global lcofgp
    if lcofgp is None : 
        rospy.loginfo("left camera detect object is none")
    translation = np.array([lcofgp.pose.position.x,lcofgp.pose.position.y,lcofgp.pose.position.z],dtype=float)
    rotation_q = np.array([lcofgp.pose.orientation.x,lcofgp.pose.orientation.y,lcofgp.pose.orientation.z,lcofgp.pose.orientation.w],dtype=float)
    R = Rotation.from_quat(rotation_q)
    T = np.eye(4,4)
    T[:3,:3] = R.as_matrix()
    T[:3,3] = translation
    T_inv = np.linalg.inv(T) # we have world to camera color optical here, need to find world to camera_base

    # fill this with the left azure rgb frame string and the left azure base frame string
    camera_translation, camera_rotation_quat = get_transformation("left_azure_rgb_camera_link","left_azure_camera_base")
    R_cam = Rotation.from_quat(camera_rotation_quat)
    T_cam = np.eye(4,4)
    T_cam[:3,:3] = R_cam.as_matrix()
    T_cam[:3,3] = camera_translation

    T_inv = T_inv@T_cam

    translation_inv = T_inv[:3,3]
    R_inv = Rotation.from_matrix(T_inv[:3,:3])
    rotation_inv_quat = R_inv.as_quat()

    # Save the transformation to a file in the ../config/ directory
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'../config/{left_camera_calibration_target_file_name}')
    transformation = {"translation": translation_inv.tolist(), "rotation": rotation_inv_quat.tolist()}
    rospy.loginfo(f"Transformation from world to left camera base frame found is {transformation}")
    transformation_json = json.dumps(transformation)
    with open(config_path, 'w') as file:
            file.write(transformation_json)

    rospy.loginfo(f"Transformation of left camera successfull, written into file config/{left_camera_calibration_target_file_name}")
    broadcast_static_transform("left_azure_camera_base","world",translation_inv,rotation_inv_quat)
    return

def calibrate_right_camera():
    global rcofgp
    if rcofgp is None : 
        rospy.loginfo("right camera detect object is none")
    translation = np.array([rcofgp.pose.position.x,rcofgp.pose.position.y,rcofgp.pose.position.z],dtype=float)
    rotation_q = np.array([rcofgp.pose.orientation.x,rcofgp.pose.orientation.y,rcofgp.pose.orientation.z,rcofgp.pose.orientation.w],dtype=float)
    R = Rotation.from_quat(rotation_q)
    T = np.eye(4,4)
    T[:3,:3] = R.as_matrix()
    T[:3,3] = translation
    T_inv = np.linalg.inv(T) # we have world to camera color optical here, need to find world to camera_base

    # fill this with the right azure rgb frame string and the right azure base frame string
    camera_translation, camera_rotation_quat = get_transformation("right_azure_rgb_camera_link","right_azure_camera_base")
    R_cam = Rotation.from_quat(camera_rotation_quat)
    T_cam = np.eye(4,4)
    T_cam[:3,:3] = R_cam.as_matrix()
    T_cam[:3,3] = camera_translation

    T_inv = T_inv@T_cam

    translation_inv = T_inv[:3,3]
    R_inv = Rotation.from_matrix(T_inv[:3,:3])
    rotation_inv_quat = R_inv.as_quat()

    # Save the transformation to a file in the ../config/ directory
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'../config/{right_camera_calibration_target_file_name}')
    transformation = {"translation": translation_inv.tolist(), "rotation": rotation_inv_quat.tolist()}
    rospy.loginfo(f"Transformation from world to right camera base frame found is {transformation}")
    transformation_json = json.dumps(transformation)
    with open(config_path, 'w') as file:
            file.write(transformation_json)

    rospy.loginfo(f"Transformation of right camera successful, written into file config/{right_camera_calibration_target_file_name}")
    broadcast_static_transform("right_azure_camera_base","world",translation_inv,rotation_inv_quat)
    return


if __name__ == "__main__":
    rospy.init_node("system_calibrator",argv=sys.argv)
    rospy.Subscriber("/collocated_camera_object_filtered_pose",PoseStamped,callback=ccofgp_cb)
    rospy.Subscriber("/left_camera_object_filtered_pose",PoseStamped,callback=lcofgp_cb)
    rospy.Subscriber("/right_camera_object_filtered_pose",PoseStamped,callback=rcofgp_cb)
    print("################################################################")
    rospy.loginfo("Waiting to detect the common frame using the collocated camera")
    rospy.wait_for_message("/collocated_camera_object_filtered_pose",PoseStamped)
    calibrate_robot()
    print("################################################################")
