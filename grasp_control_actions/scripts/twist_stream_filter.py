#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from diagnostic_updater import Updater, FunctionDiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus
import threading
import time

class TwistStreamFilter:
    def __init__(self):
        # ROS Parameters
        self.input_topic = rospy.get_param("~input_topic", "/cmd_vel_in")
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel")
        self.timeout = rospy.get_param("~timeout", 0.05)

        self.lock = threading.Lock()
        self.last_msg_time = None
        self.prev_msg_time = None
        self.proxy_input_message = None
        self.stream_active = False

        self.pub = rospy.Publisher(self.output_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, TwistStamped, self.input_cb)

        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_cb)

        # Diagnostics
        self.updater = Updater()
        self.updater.setHardwareID("twist_stream_filter")
        self.updater.add("Twist Stream Status", self.diagnostics_cb)
        rospy.Timer(rospy.Duration(1.0), lambda event: self.updater.update())

        rospy.loginfo(f"TwistStreamFilter initialized. Subscribing to {self.input_topic}, publishing to {self.output_topic}")

    def input_cb(self, msg: TwistStamped):
        with self.lock:
            now = time.time()
            self.prev_msg_time = self.last_msg_time
            self.last_msg_time = now
            self.proxy_input_message = msg

            if self.prev_msg_time is not None and (self.last_msg_time - self.prev_msg_time) <= self.timeout:
                self.stream_active = True

    def timer_cb(self, event):
        with self.lock:
            now = time.time()
            if self.last_msg_time is None or not self.stream_active:
                twist = Twist()
            elif now - self.last_msg_time > self.timeout:
                twist = Twist()
                self.stream_active = False
            else:
                twist = self.proxy_input_message.twist

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
