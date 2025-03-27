#!/usr/bin/env python
import rospy
import tf
import os
import json
from tf import transformations as tfs
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

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
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            continue

        rate.sleep()

def publish_static_transform(frame_a, frame_b, trans, rot):
    broadcaster = StaticTransformBroadcaster()
    static_transformStamped = TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = frame_a
    static_transformStamped.child_frame_id = frame_b

    static_transformStamped.transform.translation.x = trans[0]
    static_transformStamped.transform.translation.y = trans[1]
    static_transformStamped.transform.translation.z = trans[2]

    static_transformStamped.transform.rotation.x = rot[0]
    static_transformStamped.transform.rotation.y = rot[1]
    static_transformStamped.transform.rotation.z = rot[2]
    static_transformStamped.transform.rotation.w = rot[3]

    broadcaster.sendTransform(static_transformStamped)
    rospy.loginfo(f"Published static transform from {frame_a} to {frame_b}")

if __name__ == '__main__':
    try:
        frame_b = 'left_azure_camera_base'
        frame_a = 'handeye_target'
        
        trans, rot = get_transformation(frame_a, frame_b)

        # Save the transformation to a file in the ../config/ directory
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../config/transformation.json')
        transformation = {"translation": trans, "rotation": rot}
        transformation_json = json.dumps(transformation)
        
        with open(config_path, 'w') as file:
            file.write(transformation_json)

        # Publish the transformation as static
        publish_static_transform("world", frame_b, trans, rot)
        
        rospy.spin()  # Keep the node running

    except rospy.ROSInterruptException:
        pass