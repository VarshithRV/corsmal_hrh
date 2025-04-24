#!/usr/bin/env python
import rospy
import tf
from tf import transformations as tfs
import os
import json
import argparse

def get_transformation(frame_a, frame_b):
    rospy.init_node('tf_listener_node', anonymous=True)
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0) # 10 Hz
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
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            continue

def main():
    parser = argparse.ArgumentParser(description='Retrieve and save transformation between two frames.')
    parser.add_argument('source_frame', type=str, help='The source frame.')
    parser.add_argument('target_frame', type=str, help='The target frame.')
    parser.add_argument('output_filename', type=str, help='The output file to save the transformation.')
    
    args = parser.parse_args()

    try:
        trans, rot = get_transformation(args.source_frame, args.target_frame)
        # Save the transformation to the specified output file
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../config/', args.output_filename+".json")
        transformation = {"translation": trans, "rotation": rot}
        with open(config_path, 'w') as file:
            json.dump(transformation, file, indent=4)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()