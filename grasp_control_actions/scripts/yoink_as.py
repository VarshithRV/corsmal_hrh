#!/usr/bin/env python
# This should be a yoink server that will perform the grab, should be interruptible
import yaml
import rospy
import tf.transformations as tft
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Twist, Pose, TransformStamped
from scipy.spatial.transform import Rotation
import time
import numpy as np
import os
import sys
import tf2_geometry_msgs
from threading import Lock
import math


class Yoink:
    def __init__(self):
        
        self.params = rospy.get_param("/yoink_as")
        
        self.LINEAR_P_GAIN = self.params["gains"]["linear"]["p"]
        self.LINEAR_I_GAIN = self.params["gains"]["linear"]["i"]
        self.LINEAR_D_GAIN = self.params["gains"]["linear"]["d"]
        self.LINEAR_K = self.params["gains"]["linear"]["k"]
        self.ANGULAR_P_GAIN = self.params["gains"]["angular"]["p"]
        self.ANGULAR_I_GAIN = self.params["gains"]["angular"]["i"]
        self.ANGULAR_D_GAIN = self.params["gains"]["angular"]["d"]
        self.ANGULAR_K = self.params["gains"]["angular"]["k"]

        self.cmd_publish_frequency = self.params["cmd_publish_frequency"]

        self.max_linear_velocity = self.params["max_linear_velocity"]
        self.max_linear_acceleration = self.params["max_linear_acceleration"]
        self.max_angular_velocity = self.params["max_angular_velocity"]
        self.max_angular_acceleration = self.params["max_angular_acceleration"]

        self.pose_setpoint_frequency_cutoff = self.params["pose_setpoint_frequency_cuttoff"]
        self.linear_stop_threshold = self.params["linear_stop_threshold"]
        self.angular_stop_threshold = self.params["angular_stop_threshold"]
        self.pre_grasp_transform = self.params["pre_grasp_transform"]

        self.dt = 1/self.cmd_publish_frequency
        self.errorL = np.zeros((3,),dtype=float)
        self.errorO = np.array([0,0,0],dtype=float)
        self.errorLsum = np.zeros((3,),dtype=float)
        self.errorOsum = np.array([0,0,0],dtype=float)
        self.errorLprev = np.zeros((3,),dtype=float)
        self.errorOprev = np.array([0,0,0],dtype=float)

        rospy.loginfo("Started the yoink node with parameters:")
        for item in self.params:
            rospy.loginfo(f"{item} : {self.params[item]}")
        
        self.start = rospy.Time.now()
        
        self.current_velocity = None
        self.current_pose = None
        self.setpoint_velocity = None
        self.filtered_grasp_pose = None
        self.proxy_filtered_grasp_pose = None
        self.pose_message_timestamp_queue = []
        self.linear_velocity = 0.0
        self.angular_velocity  = 0.0
        self.linear_error = 0.0
        self.angular_error = 0.0

        self.mutex = Lock()
        self._now = None
        self.optimal_pose = None
        
        # pose_controller format
        self.current_pose_sub = rospy.Subscriber("/pose_controller/current_pose",PoseStamped,callback=self.current_pose_cb)
        self.current_velocity_sub = rospy.Subscriber("/pose_controller/current_velocity",Twist,callback=self.current_velocity_cb)
        self.setpoint_velocity_pub = rospy.Publisher("/pose_controller/setpoint_velocity",Twist,queue_size=1)
        self.optimal_pose_pub =  rospy.Publisher("/optimal_pose",PoseStamped,queue_size=1)
        # publish error and velocity magnitude for debug
        self.linear_error_publisher =  rospy.Publisher("/linear_error",Float32,queue_size=1)
        self.angular_error_publisher = rospy.Publisher("/angular_error",Float32,queue_size=1)
        self.linear_velocity_publisher = rospy.Publisher("/linear_velocity",Float32,queue_size=1)
        self.angular_velocity_publisher = rospy.Publisher("/angular_velocity",Float32,queue_size=1)

        # filtered grasp pose connection
        self.filtered_grasp_pose_sub = rospy.Subscriber("/filtered_grasp_pose",PoseStamped,callback=self.filtered_grasp_pose_cb)

        # Timer to update the filtered grasp pose
        filtered_grasp_pose_update_timer = rospy.Timer(rospy.Duration(0.01),callback=self.update_grasp_pose)

        # Timer to publish the optimal pose
        op_pose_publisher_timer = rospy.Timer(rospy.Duration(0.01),callback=self.publish_optimal_pose)

        # Timer to publish the error velocity
        error_velocity_publisher_timer = rospy.Timer(rospy.Duration(0.01),callback=self.publish_error_velocity)

        self._qsum  = np.zeros((4,1),dtype=float)

    def publish_error_velocity(self,event):
        self.linear_error_publisher.publish(Float32(self.linear_error))
        self.angular_error_publisher.publish(Float32(self.angular_error))
        self.linear_velocity_publisher.publish(Float32(self.linear_velocity))
        self.angular_velocity_publisher.publish(Float32(self.angular_velocity))

    # This commands the robot to go to pre grasp
    def goto_pre_grasp(self):
        if self.filtered_grasp_pose is None :
            rospy.logerr("No filtered grasp pose received")
            return

        self.errorLprev = np.zeros((3,),dtype=float)
        self.errorLsum = np.zeros((3,),dtype=float)
        self.errorOprev = np.zeros((4,),dtype=float)
        self.errorOsum = np.zeros((4,),dtype=float)


        rate = rospy.Rate(self.cmd_publish_frequency)
        while not rospy.is_shutdown() :
            
            optimal_poseL, optimal_poseO, optimal_poseQ = self.compute_pre_grasp_setpoint()
            
            current_poseL = np.array([self.current_pose.pose.position.x,self.current_pose.pose.position.y,self.current_pose.pose.position.z])
            current_poseQ = np.array([self.current_pose.pose.orientation.x,self.current_pose.pose.orientation.y,
                                      self.current_pose.pose.orientation.z,self.current_pose.pose.orientation.w])
            
            if np.linalg.norm(optimal_poseL) - np.linalg.norm(current_poseL) < self.linear_stop_threshold and np.linalg.norm(optimal_poseQ) - np.linalg.norm(current_poseQ)<self.angular_stop_threshold : 
                cmd_vel = Twist()
                print("Reached pre grasp")
                # break
            
            self.linear_error = np.linalg.norm(optimal_poseL) - np.linalg.norm(current_poseL)
            self.angular_error = np.linalg.norm(optimal_poseQ) - np.linalg.norm(current_poseQ)

            if self.filtered_grasp_pose is not None :
                cmd_vel = self.compute_cmd_vel(optimal_setpointL=optimal_poseL,optimal_setpointQ=optimal_poseQ)

            self.setpoint_velocity = cmd_vel  
            self.setpoint_velocity_pub.publish(cmd_vel)
            rate.sleep()

        self.errorLprev = np.zeros((3,1),dtype=float)
        self.errorLsum = np.zeros((3,1),dtype=float)
        self.errorOprev = np.zeros((4,1),dtype=float)
        self.errorOsum = np.zeros((4,1),dtype=float)

    # Grasp
    def grasp(self):
        self.errorLprev = np.zeros((3,1),dtype=float)
        self.errorLsum = np.zeros((3,1),dtype=float)
        self.errorOprev = np.zeros((4,1),dtype=float)
        self.errorOsum = np.zeros((4,1),dtype=float)

        if self.filtered_grasp_pose is not None:
            rate = rospy.Rate(self.cmd_publish_frequency)
            while True:
                pose_setpoint = self.filtered_grasp_pose
                pose_setpointL = [pose_setpoint.pose.position.x,pose_setpoint.pose.position.y,pose_setpoint.pose.position.z]
                pose_setpointQ = [pose_setpoint.pose.orientation.x,pose_setpoint.pose.orientation.y,pose_setpoint.pose.orientation.z,pose_setpoint.pose.orientation.w]

                current_pose = self.current_pose
                current_poseL = [current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z]
                current_poseQ = [current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w]

                if np.linalg.norm(pose_setpointL) - np.linalg.norm(current_poseL) < self.linear_stop_threshold and np.linalg.norm(pose_setpointQ) - np.linalg.norm(current_poseQ)<self.angular_stop_threshold : 
                    cmd_vel = Twist()
                    print("Reached Grasp Position")
                    # command the gripper so that it closes here
                    break

                if self.filtered_grasp_pose is not None :
                    cmd_vel = self.compute_cmd_vel(optimal_setpointL=pose_setpointL,optimal_setpointQ=pose_setpointQ)

                self.setpoint_velocity = cmd_vel
                self.setpoint_velocity_pub.publish(cmd_vel)
                rate.sleep()
            self.errorLprev = np.zeros((3,1),dtype=float)
            self.errorLsum = np.zeros((3,1),dtype=float)
            self.errorOprev = np.zeros((4,1),dtype=float)
            self.errorOsum = np.zeros((4,1),dtype=float)
        
        else : 
            rospy.logerr("Filtered grasp pose is not received")
            return
        
    # This computes the pre grasp setpoint in (linear, euler, quaterion) format
    def compute_pre_grasp_setpoint(self):
        pose_setpoint = self.filtered_grasp_pose
        pose_setpointL = np.array([pose_setpoint.pose.position.x,pose_setpoint.pose.position.y,pose_setpoint.pose.position.z])
        pose_setpointQ = np.array([pose_setpoint.pose.orientation.x,pose_setpoint.pose.orientation.y,
                                   pose_setpoint.pose.orientation.z,pose_setpoint.pose.orientation.w])
        pose_setpointO = np.array(tft.euler_from_quaternion(pose_setpointQ.tolist()))
        pose_setpointM = tft.concatenate_matrices(tft.translation_matrix(pose_setpointL), tft.quaternion_matrix(pose_setpointQ))

        transform = Pose()
        transform.position.x = self.pre_grasp_transform["position"]["x"]
        transform.position.y = self.pre_grasp_transform["position"]["y"]
        transform.position.z = self.pre_grasp_transform["position"]["z"]
        transform.orientation.x = self.pre_grasp_transform["orientation"]["x"]
        transform.orientation.y = self.pre_grasp_transform["orientation"]["y"]
        transform.orientation.z = self.pre_grasp_transform["orientation"]["z"]
        transform.orientation.w = self.pre_grasp_transform["orientation"]["w"]
        transformL = np.array([transform.position.x, transform.position.y, transform.position.z])
        transformQ = np.array([transform.orientation.x, transform.orientation.y, transform.orientation.z, transform.orientation.w])
        transformM = tft.concatenate_matrices(tft.translation_matrix(transformL), tft.quaternion_matrix(transformQ))
        
        pose_optimal_setpointM = np.dot(pose_setpointM, transformM)
        pose_optimal_setpointL = tft.translation_from_matrix(pose_optimal_setpointM)
        pose_optimal_setpointQ = tft.quaternion_from_matrix(pose_optimal_setpointM)
        pose_optimal_setpointO = tft.euler_from_quaternion(pose_optimal_setpointQ)


        # Create PoseStamped message
        pose_optimal_setpoint = PoseStamped()
        pose_optimal_setpoint.header.frame_id = "world"
        pose_optimal_setpoint.pose.position.x = pose_optimal_setpointL[0]
        pose_optimal_setpoint.pose.position.y = pose_optimal_setpointL[1]
        pose_optimal_setpoint.pose.position.z = pose_optimal_setpointL[2]
        pose_optimal_setpoint.pose.orientation.x = pose_optimal_setpointQ[0]
        pose_optimal_setpoint.pose.orientation.y = pose_optimal_setpointQ[1]
        pose_optimal_setpoint.pose.orientation.z = pose_optimal_setpointQ[2]
        pose_optimal_setpoint.pose.orientation.w = pose_optimal_setpointQ[3]

        return pose_optimal_setpointL,pose_optimal_setpointO,pose_optimal_setpointQ

    # Computes the velocity to command
    def compute_cmd_vel(self,optimal_setpointL,optimal_setpointQ):
        # check if current_pose, current_velocity, and if self.filtered_grasp_pose is being received
        if self.current_pose is None or self.current_velocity is None or self.filtered_grasp_pose is None:
            # rospy.logerr("Current pose or velociy is not received")
            return Twist()
        
        pose_current = self.current_pose
        pose_currentL = np.array([pose_current.pose.position.x,pose_current.pose.position.y,pose_current.pose.position.z])
        pose_currentO = np.array(tft.euler_from_quaternion([pose_current.pose.orientation.w,pose_current.pose.orientation.x,
                                                    pose_current.pose.orientation.y,pose_current.pose.orientation.z]))
        pose_currentQ = np.array([pose_current.pose.orientation.x,pose_current.pose.orientation.y,
                                    pose_current.pose.orientation.z,pose_current.pose.orientation.w])

        self.optimal_pose = PoseStamped()
        self.optimal_pose.header.frame_id = "world"
        self.optimal_pose.pose.position.x = optimal_setpointL[0]
        self.optimal_pose.pose.position.y = optimal_setpointL[1]
        self.optimal_pose.pose.position.z = optimal_setpointL[2]
        self.optimal_pose.pose.orientation.x = optimal_setpointQ[0]
        self.optimal_pose.pose.orientation.y = optimal_setpointQ[1]
        self.optimal_pose.pose.orientation.z = optimal_setpointQ[2]
        self.optimal_pose.pose.orientation.w = optimal_setpointQ[3]

        # Error computation
        # Note that the error orientation is in euler
        self.errorLprev = self.errorL
        self.errorOprev = self.errorO
        
        qprev = tft.quaternion_from_euler(self.errorO[0],self.errorO[1],self.errorO[2])
        q_error = tft.quaternion_multiply(optimal_setpointQ, tft.quaternion_conjugate(pose_currentQ))

        self.errorL = optimal_setpointL - pose_currentL
        self.errorO = np.array(tft.euler_from_quaternion(q_error))

        self.errorLsum = self.errorLsum + self.errorL
        self._qsum = tft.quaternion_multiply(q_error,self._qsum)
        self.errorOsum = np.array(tft.euler_from_quaternion(self._qsum.reshape((4,))))

        errorOdiff = np.array(tft.euler_from_quaternion(tft.quaternion_multiply(q_error,tft.quaternion_conjugate(qprev))))

        velocityL,velocityO = self.PID(self.errorL,self.errorLsum,self.errorL-self.errorLprev,
                                       self.errorO, self.errorOsum, errorOdiff)

        velocityL = np.array(velocityL)
        
        # clip if velocity is greater than 1m/s
        if np.linalg.norm(velocityL) > 1.0:
            velocityL /= np.linalg.norm(velocityL)

        self.linear_velocity = np.linalg.norm(velocityL)
        self.angular_velocity = np.linalg.norm(velocityO)
        # form cmd_vel message
        cmd_vel = Twist()
        cmd_vel.linear.x = velocityL[0]
        cmd_vel.linear.y = velocityL[1]
        cmd_vel.linear.z = velocityL[2]
        cmd_vel.angular.x = velocityO[0]
        cmd_vel.angular.y = velocityO[1]
        cmd_vel.angular.z = velocityO[2]
        return cmd_vel

    # publish optimal pose to debug
    def publish_optimal_pose(self,event):
        if self.optimal_pose is not None :
            self.optimal_pose_pub.publish(self.optimal_pose)

    # compute velocityL and velocityO from errorL and errorO
    def PID(self,errorL,errorLsum,errorLdiff,errorO,errorOsum,errorOdiff): 
        velocityL = self.LINEAR_K* ((self.LINEAR_P_GAIN*errorL) + (self.LINEAR_I_GAIN*errorLsum*self.dt) + (self.LINEAR_D_GAIN*(errorLdiff/self.dt)))
        velocityO = self.ANGULAR_K* ((self.ANGULAR_P_GAIN*errorO) + (self.ANGULAR_I_GAIN*errorOsum*self.dt) + (self.ANGULAR_D_GAIN*(errorOdiff/self.dt)))
        return velocityL, velocityO

    # callback functions below
    def current_pose_cb(self,msg: PoseStamped):
        self.current_pose = msg

    def current_velocity_cb(self,msg: Twist):
        self.current_velocity = msg

    def filtered_grasp_pose_cb(self,msg: PoseStamped):
        with self.mutex : 
            self._now = time.time()
            self.pose_message_timestamp_queue.append(self._now)
            self.proxy_filtered_grasp_pose = msg
            
    # update grasp pose if message frequency is greater than the threshold
    def update_grasp_pose(self,event):
        with self.mutex : 
            self.pose_message_timestamp_queue = [t for t in self.pose_message_timestamp_queue if time.time() - t <= 1]
            if len(self.pose_message_timestamp_queue) < self.pose_setpoint_frequency_cutoff:
                self.filtered_grasp_pose = None
            else :
                self.filtered_grasp_pose = self.proxy_filtered_grasp_pose 


if __name__ == "__main__":
    rospy.init_node("radial_tracker")
    yoink = Yoink()
    rospy.sleep(0.9)
    yoink.goto_pre_grasp()
    # yoink.grasp()
    rospy.spin()