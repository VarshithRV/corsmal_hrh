#!/usr/bin/env python3

import rospy
import argparse
import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import threading

class PoseStatsTracker:
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.lock = threading.Lock()
        self.positions = []
        self.quaternions = []

        rospy.Subscriber(self.topic_name, PoseStamped, self.pose_callback)
        rospy.Timer(rospy.Duration(3.0), self.timer_callback)

        rospy.loginfo(f"Subscribed to topic: {self.topic_name}")

    def pose_callback(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation

        with self.lock:
            self.positions.append([pos.x, pos.y, pos.z])
            self.quaternions.append([ori.x, ori.y, ori.z, ori.w])

    def timer_callback(self, event):
        with self.lock:
            if len(self.positions) < 2:
                rospy.loginfo("Waiting for more pose data...")
                return

            positions_np = np.array(self.positions)
            pos_std = np.std(positions_np, axis=0)

            # Compute angular deviation from first pose
            q_ref = R.from_quat(self.quaternions[0])
            angle_diffs = []

            for q in self.quaternions:
                q_rel = q_ref.inv() * R.from_quat(q)
                angle_diffs.append(q_rel.magnitude())

            rot_std = np.std(angle_diffs)

        rospy.loginfo("\n--- Pose Standard Deviations ---")
        rospy.loginfo(f"Linear STD (m):  x={pos_std[0]:.4f}, y={pos_std[1]:.4f}, z={pos_std[2]:.4f}")
        rospy.loginfo(f"Angular STD (rad):  {rot_std:.4f}")

def main():
    parser = argparse.ArgumentParser(
        description="ROS node to compute running standard deviation of PoseStamped topic.\n"
                    "Example: rosrun your_package pose_std_tracker.py /your/pose_topic",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument('topic', type=str, help='PoseStamped topic to subscribe to (e.g. /robot_pose)')
    args, unknown = parser.parse_known_args()  # To ignore ROS-specific args

    rospy.init_node('pose_std_tracker', anonymous=True)
    PoseStatsTracker(args.topic)
    rospy.spin()

if __name__ == '__main__':
    main()
