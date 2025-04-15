#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from collections import defaultdict
import numpy as np
from scipy import stats

class PoseStatsCollector:
    def __init__(self):
        rospy.init_node("pose_statistics_collector", anonymous=True)

        self.topic_name = "/pose_statistics_input_topic"
        self.log_interval = 1.0  # in seconds

        self.data = defaultdict(list)

        self.sub = rospy.Subscriber(self.topic_name, PoseStamped, self.pose_callback)
        self.timer = rospy.Timer(rospy.Duration(self.log_interval), self.log_statistics)

        rospy.loginfo(f"Subscribed to {self.topic_name}, logging every {self.log_interval} seconds.")

    def pose_callback(self, msg):
        # Linear
        self.data["position_x"].append(msg.pose.position.x)
        self.data["position_y"].append(msg.pose.position.y)
        self.data["position_z"].append(msg.pose.position.z)

        # Orientation
        self.data["orientation_w"].append(msg.pose.orientation.w)
        self.data["orientation_x"].append(msg.pose.orientation.x)
        self.data["orientation_y"].append(msg.pose.orientation.y)
        self.data["orientation_z"].append(msg.pose.orientation.z)

    def log_statistics(self, event):
        if not self.data["position_x"]:
            rospy.logwarn("No data received yet.")
            return

        for key, values in self.data.items():
            array = np.array(values)
            mean = np.mean(array)
            median = np.median(array)
            try:
                mode = stats.mode(array, keepdims=False).mode
            except:
                mode = "N/A"
            std_dev = np.std(array)
            variance = np.var(array)

            rospy.loginfo(f"{key}: mean={mean:.4f}, median={median:.4f}, mode={mode}, std_dev={std_dev:.4f}, variance={variance:.4f}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        PoseStatsCollector().run()
    except rospy.ROSInterruptException:
        pass
