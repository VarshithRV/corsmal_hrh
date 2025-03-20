#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
from threading import Lock
import time
import tf.transformations as tf_trans

INITIAL_POSE = PoseStamped()
INITIAL_POSE.header.frame_id = "world"
INITIAL_POSE.pose.position.x = 0.0
INITIAL_POSE.pose.position.y = 0.0
INITIAL_POSE.pose.position.z = 0.0
INITIAL_POSE.pose.orientation.x = 0.0
INITIAL_POSE.pose.orientation.y = 0.0
INITIAL_POSE.pose.orientation.z = 0.0
INITIAL_POSE.pose.orientation.w = 1.0

LINEAR_ACCELERATION = 3.0  # m/s²
ANGULAR_ACCELERATION = 1.0  # rad/s²
THRESHOLD = 0.01
P_GAIN = 1.0

class PoseController:
    def __init__(self):
        rospy.init_node("pose_controller_node")

        self.current_pose = INITIAL_POSE
        self.setpoint_velocity = Twist()
        self.current_velocity = Twist()

        self.last_update_time = rospy.Time.now()

        self.velocity_lock = Lock()
        self.message_timestamps = []
        self.latest_setpoint_msg = Twist()

        self.current_pose_publisher = rospy.Publisher("/pose_controller/current_pose", PoseStamped, queue_size=1)
        self.current_velocity_publisher = rospy.Publisher("/pose_controller/current_velocity", Twist, queue_size=1)

        self.setpoint_velocity_sub = rospy.Subscriber("/pose_controller/setpoint_velocity", Twist, self.setpoint_vel_cb)

        rospy.Timer(rospy.Duration(0.05), self.update_controller)  # 20Hz loop

    def setpoint_vel_cb(self, msg):
        with self.velocity_lock:
            now = time.time()
            self.message_timestamps.append(now)
            self.message_timestamps = [t for t in self.message_timestamps if now - t <= 1.0]
            self.latest_setpoint_msg = msg

    def update_controller(self, event):
        dt = (rospy.Time.now() - self.last_update_time).to_sec()
        if dt == 0:
            return
        self.last_update_time = rospy.Time.now()

        now = time.time()
        with self.velocity_lock:
            self.message_timestamps = [t for t in self.message_timestamps if now - t <= 1.0]
            msg_rate = len(self.message_timestamps)

            if msg_rate > 2:
                self.setpoint_velocity = self.latest_setpoint_msg
            else:
                self.setpoint_velocity = Twist()

        # Compute current_velocity with acceleration constraints
        cur_vel = np.array([
            self.current_velocity.linear.x,
            self.current_velocity.linear.y,
            self.current_velocity.linear.z,
            self.current_velocity.angular.x,
            self.current_velocity.angular.y,
            self.current_velocity.angular.z
        ])

        tgt_vel = np.array([
            self.setpoint_velocity.linear.x,
            self.setpoint_velocity.linear.y,
            self.setpoint_velocity.linear.z,
            self.setpoint_velocity.angular.x,
            self.setpoint_velocity.angular.y,
            self.setpoint_velocity.angular.z
        ])

        vel_diff = tgt_vel - cur_vel

        for i in range(3):  # Linear components
            if abs(vel_diff[i]) > THRESHOLD:
                change = np.clip(vel_diff[i], -LINEAR_ACCELERATION * dt, LINEAR_ACCELERATION * dt)
                cur_vel[i] += change
            else:
                cur_vel[i] = tgt_vel[i]

        for i in range(3, 6):  # Angular components
            if abs(vel_diff[i]) > THRESHOLD:
                change = np.clip(vel_diff[i], -ANGULAR_ACCELERATION * dt, ANGULAR_ACCELERATION * dt)
                cur_vel[i] += change
            else:
                cur_vel[i] = tgt_vel[i]

        # Update current_velocity message
        self.current_velocity.linear.x = cur_vel[0]
        self.current_velocity.linear.y = cur_vel[1]
        self.current_velocity.linear.z = cur_vel[2]
        self.current_velocity.angular.x = cur_vel[3]
        self.current_velocity.angular.y = cur_vel[4]
        self.current_velocity.angular.z = cur_vel[5]

        self.current_velocity_publisher.publish(self.current_velocity)

        # Update position using current linear velocity
        self.current_pose.pose.position.x += cur_vel[0] * dt * P_GAIN
        self.current_pose.pose.position.y += cur_vel[1] * dt * P_GAIN
        self.current_pose.pose.position.z += cur_vel[2] * dt * P_GAIN

        # Update orientation using angular velocity (integrate rotation)
        q = [
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        ]

        angular_vel = cur_vel[3:6]
        angle = np.linalg.norm(angular_vel) * dt * P_GAIN

        if angle > 0.0:
            axis = angular_vel / np.linalg.norm(angular_vel)
            delta_q = tf_trans.quaternion_about_axis(angle, axis)
            updated_q = tf_trans.quaternion_multiply(q, delta_q)
            self.current_pose.pose.orientation.x = updated_q[0]
            self.current_pose.pose.orientation.y = updated_q[1]
            self.current_pose.pose.orientation.z = updated_q[2]
            self.current_pose.pose.orientation.w = updated_q[3]

        self.current_pose.header.stamp = rospy.Time.now()
        self.current_pose_publisher.publish(self.current_pose)

if __name__ == "__main__":
    try:
        PoseController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
