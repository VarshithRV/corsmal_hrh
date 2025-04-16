#!/usr/bin/env python3
import rospy
import tf2_ros
import tf
import numpy as np
from geometry_msgs.msg import Twist, TwistStamped
from diagnostic_updater import Updater
from diagnostic_msgs.msg import DiagnosticStatus
import threading
import time


class TwistStreamFilter:
    def __init__(self):
        # ROS Parameters
        self.input_topic = rospy.get_param("~input_topic", "/cmd_vel_in")
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel")
        self.timeout = rospy.get_param("~timeout", 0.05)
        self.source_frame = rospy.get_param("~source_frame", "base_link")
        self.target_frame = rospy.get_param("~target_frame", "world")

        self.lock = threading.Lock()
        self.last_msg_time = None
        self.prev_msg_time = None
        self.proxy_input_message = None
        self.stream_active = False

        self.pub = rospy.Publisher(self.output_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, TwistStamped, self.input_cb)

        self.rotation_matrix = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self._wait_for_static_rotation()

        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_cb)

        # Diagnostics
        self.updater = Updater()
        self.updater.setHardwareID("twist_stream_filter")
        self.updater.add("Twist Stream Status", self.diagnostics_cb)
        rospy.Timer(rospy.Duration(1.0), lambda event: self.updater.update())

        rospy.loginfo(f"TwistStreamFilter initialized. Subscribing to {self.input_topic}, publishing to {self.output_topic}")
        rospy.loginfo(f"Transforming Twist from '{self.source_frame}' to '{self.target_frame}'")

    def _wait_for_static_rotation(self):
        rospy.loginfo(f"Waiting for static transform from {self.source_frame} to {self.target_frame}...")

        while not rospy.is_shutdown() and self.rotation_matrix is None:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    self.source_frame,
                    rospy.Time(0),
                    rospy.Duration(1.0)
                )
                quat = tf_msg.transform.rotation
                self.rotation_matrix = tf.transformations.quaternion_matrix(
                    [quat.x, quat.y, quat.z, quat.w]
                )[:3, :3]
                rospy.loginfo("Static transform acquired.")
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
                rospy.logerr_throttle(5.0, f"Waiting for static transform: {e}")
                rospy.sleep(1.0)

    def input_cb(self, msg: TwistStamped):
        with self.lock:
            now = time.time()
            self.prev_msg_time = self.last_msg_time
            self.last_msg_time = now
            self.proxy_input_message = msg

            if self.prev_msg_time is not None and (self.last_msg_time - self.prev_msg_time) <= self.timeout:
                self.stream_active = True

    def transform_twist(self, twist_stamped: TwistStamped) -> Twist:
        if self.rotation_matrix is None:
            return Twist()  # Should not happen since we block until available

        lin = twist_stamped.twist.linear
        ang = twist_stamped.twist.angular

        lin_vec = np.array([lin.x, lin.y, lin.z])
        ang_vec = np.array([ang.x, ang.y, ang.z])

        lin_transformed = self.rotation_matrix @ lin_vec
        ang_transformed = self.rotation_matrix @ ang_vec

        twist_out = Twist()
        twist_out.linear.x, twist_out.linear.y, twist_out.linear.z = lin_transformed
        twist_out.angular.x, twist_out.angular.y, twist_out.angular.z = ang_transformed

        return twist_out

    def timer_cb(self, event):
        with self.lock:
            now = time.time()
            if self.rotation_matrix is None:
                # Don't publish anything if transform is not ready
                return

            if self.last_msg_time is None or not self.stream_active:
                twist = Twist()
            elif now - self.last_msg_time > self.timeout:
                twist = Twist()
                self.stream_active = False
            else:
                twist = self.transform_twist(self.proxy_input_message)

        self.pub.publish(twist)

    def diagnostics_cb(self, stat):
        with self.lock:
            if self.last_msg_time is None:
                return stat.summary(DiagnosticStatus.WARN, "No messages received yet")

            age = time.time() - self.last_msg_time
            if age > self.timeout:
                return stat.summary(DiagnosticStatus.ERROR, "Twist stream inactive")
            else:
                return stat.summary(DiagnosticStatus.OK, "Twist stream active")


if __name__ == '__main__':
    rospy.init_node("twist_stream_filter")
    node = TwistStreamFilter()
    rospy.spin()
