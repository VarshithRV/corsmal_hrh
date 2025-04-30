#!/usr/bin/env python3

'''
filter has the following responsibilities:
1. timer watchdog
2. transformation to velocity controller planning frame
3. linear filter over input velocity, only applied for linear part of the velocity and not the angular
'''
import rospy
import tf2_ros
import tf
import numpy as np
import tf.transformations as tft
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from geometry_msgs.msg import Twist, TwistStamped
from diagnostic_updater import Updater
from diagnostic_msgs.msg import DiagnosticStatus
import threading
import time


class TwistStreamFilter:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/cmd_vel_in")
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel")
        self.output_topic_type = rospy.get_param("~output_topic_type", "Twist")
        self.timeout = rospy.get_param("~timeout", 0.05)
        self.source_frame = rospy.get_param("~source_frame", "world")
        self.target_frame = rospy.get_param("~target_frame", "base")
        self.alpha = rospy.get_param("~alpha",0.25) #more means trust filtered twist more
        self.twist_out = None

        self.lock = threading.Lock()
        self.last_msg_time = None
        self.prev_msg_time = None
        self.proxy_input_message = None
        self.filtered_command_vel = None
        self.stream_active = False

        self.pub = None
        self.command_vel = None
        if self.output_topic_type == "Twist":
            self.pub = rospy.Publisher(self.output_topic, Twist, queue_size=10)
            self.command_vel = Twist()
        if self.output_topic_type == "TwistStamped":
            self.pub = rospy.Publisher(self.output_topic, TwistStamped, queue_size=10)
            self.command_vel = TwistStamped()
        self.sub = rospy.Subscriber(self.input_topic, TwistStamped, self.input_cb)

        self.rotation_matrix = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self._wait_for_static_rotation()

        self.publisher_timer = rospy.Timer(rospy.Duration(0.02), self.output_publisher)
        self.stream_active_checker_timer = rospy.Timer(rospy.Duration(0.02), self.stream_active_checker)
        self.stream_linear_filter_timer = rospy.Timer(rospy.Duration(0.02),self.stream_linear_filter)
        self.compute_command_vel_timer = rospy.Timer(rospy.Duration(0.02),self.compute_command_vel)

        # Diagnostics
        self.updater = Updater()
        self.updater.setHardwareID("twist_stream_filter")
        self.updater.add("Twist Stream Status", self.diagnostics_cb)
        rospy.Timer(rospy.Duration(1.0), lambda event: self.updater.update())

        rospy.loginfo(f"TwistStreamFilter initialized. Subscribing to {self.input_topic}, publishing to {self.output_topic}")
        rospy.loginfo(f"Transforming Twist from '{self.source_frame}' to '{self.target_frame}'")

    def stream_linear_filter(self,event):
        if self.proxy_input_message is None :
            return
        # apply linear filter with lerp and slerp on velocity, update in self.filtered_proxy_input_message(PoseStamped)
        linear = np.array([self.command_vel.twist.linear.x,self.command_vel.twist.linear.y,self.command_vel.twist.linear.z],dtype=float)
        angular = np.array([self.command_vel.twist.angular.x,self.command_vel.twist.angular.y,self.command_vel.twist.angular.z],dtype =float)
        
        filtered_linear = np.zeros((3,),dtype=float)
        filtered_angular = np.zeros((3,),dtype=float)
        
        if self.filtered_command_vel is not None :
            # apply filter logic for linear and angular here
            if self.output_topic_type == "TwistStamped":
                filtered_linear = np.array([self.filtered_command_vel.twist.linear.x,self.filtered_command_vel.twist.linear.y,self.filtered_command_vel.twist.linear.z],dtype=float)
                filtered_angular = np.array([self.filtered_command_vel.twist.angular.x,self.filtered_command_vel.twist.angular.y,self.filtered_command_vel.twist.angular.z],dtype =float)
            if self.output_topic_type == "Twist":
                filtered_linear = np.array([self.filtered_command_vel.linear.x,self.filtered_command_vel.linear.y,self.filtered_command_vel.linear.z],dtype=float)
                filtered_angular = np.array([self.filtered_command_vel.angular.x,self.filtered_command_vel.angular.y,self.filtered_command_vel.angular.z],dtype =float)
            filtered_linear = self.lerp(linear,filtered_linear,self.alpha)
            filtered_angular = self.slerp(angular,filtered_angular,self.alpha)

        if self.output_topic_type == "TwistStamped":
            self.filtered_command_vel = TwistStamped()
            self.filtered_command_vel.header.frame_id = self.source_frame
            self.filtered_command_vel.twist.linear.x = filtered_linear[0]
            self.filtered_command_vel.twist.linear.y = filtered_linear[1]
            self.filtered_command_vel.twist.linear.z = filtered_linear[2]
            self.filtered_command_vel.twist.angular.x = filtered_angular[0]
            self.filtered_command_vel.twist.angular.y = filtered_angular[1]
            self.filtered_command_vel.twist.angular.z = filtered_angular[2]
        if self.output_topic_type == "Twist":
            self.filtered_command_vel = Twist()
            self.filtered_command_vel.linear.x = filtered_linear[0]
            self.filtered_command_vel.linear.y = filtered_linear[1]
            self.filtered_command_vel.linear.z = filtered_linear[2]
            self.filtered_command_vel.angular.x = filtered_angular[0]
            self.filtered_command_vel.angular.y = filtered_angular[1]
            self.filtered_command_vel.angular.z = filtered_angular[2]
    
    def lerp(self,input,impulse,alpha):
        # return (1-alpha)*input + alpha*impulse
        return input

    def slerp(self, input: np.ndarray, impulse: np.ndarray, alpha: float) -> np.ndarray:
        return input
        # # Compute magnitudes
        # norm_input = np.linalg.norm(input)
        # norm_impulse = np.linalg.norm(impulse)

        # if norm_input < 1e-6 and norm_impulse < 1e-6:
        #     return np.zeros(3)
        # elif norm_input < 1e-6:
        #     return alpha * impulse
        # elif norm_impulse < 1e-6:
        #     return (1 - alpha) * input

        # # Normalize vectors to get directions
        # dir_input = input / norm_input
        # dir_impulse = impulse / norm_impulse

        # # Construct quaternions that rotate from Z to the direction vector
        # def vector_to_quaternion(v):
        #     # Assumes input is normalized
        #     axis = np.cross([0, 0, 1], v)
        #     axis_norm = np.linalg.norm(axis)
        #     if axis_norm < 1e-6:
        #         # Parallel to Z
        #         if np.dot([0, 0, 1], v) > 0:
        #             return tft.quaternion_about_axis(0, [1, 0, 0])
        #         else:
        #             return tft.quaternion_about_axis(np.pi, [1, 0, 0])
        #     axis = axis / axis_norm
        #     angle = np.arccos(np.clip(np.dot([0, 0, 1], v), -1.0, 1.0))
        #     return tft.quaternion_about_axis(angle, axis)

        # q1 = vector_to_quaternion(dir_input)
        # q2 = vector_to_quaternion(dir_impulse)

        # # SLERP between q1 and q2
        # dot = np.dot(q1, q2)

        # # If dot < 0, the interpolation will take the long way around the sphere.
        # # To fix this, invert one quaternion.
        # if dot < 0.0:
        #     q2 = -q2
        #     dot = -dot

        # DOT_THRESHOLD = 0.9995
        # if dot > DOT_THRESHOLD:
        #     # Linear interpolation fallback
        #     q_interp = (1 - alpha) * q1 + alpha * q2
        #     q_interp = q_interp / np.linalg.norm(q_interp)
        # else:
        #     theta_0 = np.arccos(dot)
        #     sin_theta_0 = np.sin(theta_0)
        #     theta = theta_0 * alpha
        #     sin_theta = np.sin(theta)

        #     s0 = np.sin(theta_0 - theta) / sin_theta_0
        #     s1 = sin_theta / sin_theta_0
        #     q_interp = s0 * q1 + s1 * q2

        # # Convert quaternion back to rotation vector
        # # axis, angle = tft.rotation_from_matrix(tft.quaternion_matrix(q_interp))
        # angle, axis,_ = tft.rotation_from_matrix(tft.quaternion_matrix(q_interp))
        # dir_rotated = axis * angle

        # # Normalize and rescale with interpolated magnitude
        # dir_rotated_normalized = dir_rotated / np.linalg.norm(dir_rotated)
        # interpolated_magnitude = (1 - alpha) * norm_input + alpha * norm_impulse

        # return dir_rotated_normalized * interpolated_magnitude


    def stream_active_checker(self, event):
        with self.lock:
            if self.prev_msg_time is not None: # check if atleast two messages have been received
                if rospy.get_time() - self.last_msg_time <=0.05: # check if the last message received is within time out
                    if self.last_msg_time - self.prev_msg_time <= self.timeout: # the messages have greater frequency
                        self.stream_active=True
                    else :
                        self.stream_active=False
                else:
                    self.stream_active=False
            else:
                self.stream_active=False
        

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

    def transform_twist(self, twist_stamped: TwistStamped):
        if self.rotation_matrix is None:
            if self.output_topic_type == "Twist":
                return Twist()
            if self.output_topic_type == "TwistStamped":
                return TwistStamped()
        
        lin = twist_stamped.twist.linear
        ang = twist_stamped.twist.angular

        lin_vec = np.array([lin.x, lin.y, lin.z])
        ang_vec = np.array([ang.x, ang.y, ang.z])

        lin_transformed = self.rotation_matrix @ lin_vec
        ang_transformed = self.rotation_matrix @ ang_vec

        if self.output_topic_type == "Twist":
            twist_out = Twist()
            twist_out.linear.x, twist_out.linear.y, twist_out.linear.z = lin_transformed
            twist_out.angular.x, twist_out.angular.y, twist_out.angular.z = ang_transformed
            self.twist_out = twist_out
        if self.output_topic_type == "TwistStamped":
            twist_out = TwistStamped()
            twist_out.header.frame_id = self.target_frame
            twist_out.twist.linear.x, twist_out.twist.linear.y, twist_out.twist.linear.z = lin_transformed
            twist_out.twist.angular.x, twist_out.twist.angular.y, twist_out.twist.angular.z = ang_transformed
            self.twist_out = twist_out
        return self.twist_out

    def compute_command_vel(self, event):
        with self.lock:
            if self.rotation_matrix is None:
                return
            if self.stream_active:
                self.command_vel = self.transform_twist(self.proxy_input_message)
            else:
                if self.output_topic_type == "Twist":
                    self.command_vel = Twist()
                if self.output_topic_type == "TwistStamped":
                    self.command_vel = TwistStamped()
                    self.command_vel.header.frame_id == self.target_frame

    def output_publisher(self, event):
        if self.command_vel is not None:
            self.pub.publish(self.command_vel)

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
