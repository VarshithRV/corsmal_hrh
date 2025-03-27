#!/usr/bin/env python
import rospy
import json
import os
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

def read_transformation(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
        return data['translation'], data['rotation']

def publish_static_transform(frame_a, frame_b, trans, rot):
    rospy.init_node('static_tf_publisher_node', anonymous=True)
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
        # Define frame names
        frame_a = 'world'
        frame_b = 'right_azure_camera_base'
        
        # Define the path to the transformation JSON file
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../config/right_azure_extrinsic.json')
        
        # Read the transformation from the file
        trans, rot = read_transformation(config_path)

        # Publish the transformation as static
        publish_static_transform(frame_a, frame_b, trans, rot)

        rospy.spin()  # Keep the node running

    except rospy.ROSInterruptException:
        pass