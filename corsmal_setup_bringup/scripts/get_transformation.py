#!/usr/bin/env python

import rospy
import tf
from tf import transformations as tfs
import os
import json

def get_transformation(frame_a, frame_b):
    rospy.init_node('tf_listener_node', anonymous=True)

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
            euler = tfs.euler_from_quaternion(rot)
            rospy.loginfo(f"Rotation (euler): {euler}")
            return trans, rot
            break

        except Exception as e:
            continue


if __name__ == '__main__':
    try:
        frame_b = 'rightcamera_base'
        frame_a = 'handeye_target'
        trans,rot = get_transformation(frame_a, frame_b)
        # Save the transformation to a file in the ../config/ directory
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../config/transformation.json')
        transformation = {"translation":trans,"rotation":rot}
        transformation_json = json.dumps(transformation)
        with open(config_path, 'w') as file:
            file.write(transformation_json)
    except rospy.ROSInterruptException:
        pass