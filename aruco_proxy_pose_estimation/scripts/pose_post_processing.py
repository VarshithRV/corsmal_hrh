import rospy
import numpy as np
import sys
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf.transformations import quaternion_matrix, quaternion_from_matrix

class PosePostProcess:
    def __init__(self):
        self.parent_frame = rospy.get_param("~parent_frame", "base_link")  # parent frame
        self.X = rospy.get_param("~X", 42.0) / 1000  # marker X in mm
        self.Y = rospy.get_param("~Y", 42.0) / 1000  # marker Y in mm
        self.O_d = rospy.get_param("~D", 42.0) / 1000  # object D in mm
        self.P = rospy.get_param("~P", 0.0) / 1000  # padding in mm
        self.input_pose_topic = rospy.get_param("~input_pose_topic", "/object_filtered_pose")
        self.output_pose_topic = rospy.get_param("~output_pose_topic", "/filtered_grasp_pose")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # object to grasp pose transformation matrix
        self.object_to_grasp_T = np.array(
            [
                [0, 0, 1, self.X / 2 - self.P],
                [0, 1,  0, self.Y / 2],
                [-1, 0,  0, -self.O_d / 2],
                [0, 0,  0, 1]
            ], dtype=float
        )

        rospy.Subscriber(self.input_pose_topic, PoseStamped, callback=self.input_pose_cb)
        self.pose_publisher = rospy.Publisher(self.output_pose_topic, PoseStamped, queue_size=10)
        

    def pose_to_matrix(self, pose):
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        T = quaternion_matrix(quat)
        T[0, 3] = pose.position.x
        T[1, 3] = pose.position.y
        T[2, 3] = pose.position.z
        return T

    def matrix_to_pose(self, T, frame_id):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_id
        trans = T[:3, 3]
        quat = quaternion_from_matrix(T)
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def transform_pose(self, pose, target_frame):
        try:
            return self.tf_buffer.transform(pose, target_frame, rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transform failed: {e}")
            return None

    def input_pose_cb(self, msg):
        input_T = self.pose_to_matrix(msg.pose)
        grasp_T = np.dot(input_T, self.object_to_grasp_T)
        grasp_pose_local = self.matrix_to_pose(grasp_T, msg.header.frame_id)
        grasp_pose_world = self.transform_pose(grasp_pose_local, self.parent_frame)
        grasp_pose_world.header.frame_id = self.parent_frame
        if grasp_pose_world:
            self.pose_publisher.publish(grasp_pose_world)

if __name__ == "__main__":
    rospy.init_node("pose_post_processing", argv=sys.argv)
    try:
        processor = PosePostProcess()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()