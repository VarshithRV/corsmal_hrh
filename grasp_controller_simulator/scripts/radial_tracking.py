#!/usr/bin/env python
#Radial tracking code
import yaml
import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, Twist
from scipy.spatial.transform import Rotation
import time
import numpy as np
import sys
from threading import Lock
import math

class RadialTracker:
    def __init__(self):
        self.config_file = '/home/barracuda/corsmal_ws/src/corsmal_hrh/grasp_controller_simulator/scripts/config/radial_tracking.yaml'
        
        self.params = self.load_parameters(self.config_file)
        
        if self.params is None : 
            rospy.loginfo("Parameters not imported, shutting down")
            sys.exit()
        else :
            rospy.loginfo('Parameter imported successfully\n Node started with configuration :\n %s'%self.params)


        self.duration = self.params["duration"]
        
        self.ready_pose = PoseStamped()
        self.ready_pose.header.frame_id = "world"
        self.ready_pose.pose.position.x = self.params["ready_pose"]["position"]["x"]
        self.ready_pose.pose.position.y = self.params["ready_pose"]["position"]["y"]
        self.ready_pose.pose.position.z = self.params["ready_pose"]["position"]["z"]
        self.ready_pose.pose.orientation.w = self.params["ready_pose"]["orientation"]["w"]
        self.ready_pose.pose.orientation.x = self.params["ready_pose"]["orientation"]["x"]
        self.ready_pose.pose.orientation.y = self.params["ready_pose"]["orientation"]["y"]
        self.ready_pose.pose.orientation.z = self.params["ready_pose"]["orientation"]["z"]
        
        self.LINEAR_P_GAIN = self.params["LINEAR_P_GAIN"]
        self.LINEAR_I_GAIN = self.params["LINEAR_I_GAIN"]
        self.LINEAR_D_GAIN = self.params["LINEAR_D_GAIN"]
        self.LINEAR_K = self.params["LINEAR_K_GAIN"]
        self.ANGULAR_P_GAIN = self.params["ANGULAR_P_GAIN"]
        self.ANGULAR_I_GAIN = self.params["ANGULAR_I_GAIN"]
        self.ANGULAR_D_GAIN = self.params["ANGULAR_D_GAIN"]
        self.ANGULAR_K = self.params["ANGULAR_K_GAIN"]

        self.cmd_publish_frequency = self.params["cmd_publish_frequency"]
        self.dt = 1/self.cmd_publish_frequency
        self.errorL = np.array([0.0,0.0,0.0])
        self.errorO = np.array([0.0,0.0,0.0])
        self.errorLsum = np.array([0.0,0.0,0.0])
        self.errorOsum = np.array([0.0,0.0,0.0])
        self.errorLprev = np.array([0.0,0.0,0.0])
        self.errorOprev = np.array([0.0,0.0,0.0])

        self.max_linear_velocity = self.params["max_linear_velocity"]
        self.max_linear_acceleration = self.params["max_linear_acceleration"]
        self.max_angular_velocity = self.params["max_angular_velocity"]
        self.max_angular_acceleration = self.params["max_angular_acceleration"]

        self.pose_setpoint_frequency_cutoff = self.params["pose_setpoint_frequency_cuttoff"]

        self.transformation_interpolation_coefficient = self.params["transformation_interpolation_coefficient"]

        self.start = rospy.Time.now()
        
        self.current_velocity = None
        self.current_pose = None
        self.setpoint_velocity = None
        self.filtered_grasp_pose = None
        self.proxy_filtered_grasp_pose = None
        self.pose_message_timestamp_queue = []

        self.mutex = Lock()
        self._now = None
        self.optimal_pose = None
        
        # pose_controller format
        self.current_pose_sub = rospy.Subscriber("/pose_controller/current_pose",PoseStamped,callback=self.current_pose_cb)
        self.current_velocity_sub = rospy.Subscriber("/pose_controller/current_velocity",Twist,callback=self.current_velocity_cb)
        self.setpoint_velocity_pub = rospy.Publisher("/pose_controller/setpoint_velocity",Twist,queue_size=1)
        self.optimal_pose_pub =  rospy.Publisher("/pose_controller/optimal_pose",PoseStamped,queue_size=1)

        # filtered grasp pose connection
        self.filtered_grasp_pose_sub = rospy.Subscriber("/filtered_grasp_pose",PoseStamped,callback=self.filtered_grasp_pose_cb)

        # Timer to publish the command velocity
        cmd_vel_timer = rospy.Timer(rospy.Duration(self.dt),callback=self.command_velocity)

        # Timer to update the filtered grasp pose
        filtered_grasp_pose_update_timer = rospy.Timer(rospy.Duration(0.01),callback=self.update_grasp_pose)

        # Timer to publish the optimal pose
        op_pose_publisher_timer = rospy.Timer(rospy.Duration(0.01),callback=self.publish_optimal_pose)

    def publish_optimal_pose(self,event):
        if self.optimal_pose is not None :
            self.optimal_pose_pub.publish(self.optimal_pose)

    def command_velocity(self,event):
        start = rospy.Time.now()
        while rospy.Time.now().to_sec() - start.to_sec() <= self.duration:
            if self.filtered_grasp_pose is not None :
                cmd_vel = self.compute_cmd_vel()
                self.setpoint_velocity_pub.publish(cmd_vel)

    def compute_optimal_setpoint(self):
        pose_setpoint = self.filtered_grasp_pose
        pose_setpointL = np.array([pose_setpoint.pose.position.x,pose_setpoint.pose.position.y,pose_setpoint.pose.position.z])
    
        pose_current = self.current_pose
        
        pose_ready = self.ready_pose
        pose_readyL = np.array([pose_ready.pose.position.x,pose_ready.pose.position.y,pose_ready.pose.position.z])
        direction = pose_setpointL-pose_readyL
        unit_direction = direction/(np.linalg.norm(direction))
        pose_optimal_setpointL = pose_readyL + self.transformation_interpolation_coefficient*unit_direction
        
        # vector pointing to the grasp pose from the pose_optimal_setpointL
        unit_direction = (pose_optimal_setpointL - pose_setpointL)/(np.linalg.norm(pose_setpointL - pose_optimal_setpointL))

        
        # # method 1 for finding pose_optimal_setpointO
        # pose_optimal_setpointQ = list(np.append(1,math.sin(math.pi/2)*unit_direction)) # create a quaternion # this might not be a good 
        # pose_optimal_setpointO = np.array(tft.euler_from_quaternion(pose_optimal_setpointQ))
        
        # # method 2
        # x_axis=unit_direction
        # up=np.array([0, 0, 1])
        # z_axis=np.cross(x_axis, up)
        # if np.linalg.norm(z_axis) < 1e-6:
        #     z_axis = np.array([0, 0, 1])
        # z_axis /= np.linalg.norm(z_axis)
        # y_axis = np.cross(z_axis, x_axis)
        # y_axis /= np.linalg.norm(y_axis)

        # # Rotation matrix with x_axis as first column
        # rot_matrix = np.eye(4)
        # rot_matrix[0:3, 0] = x_axis
        # rot_matrix[0:3, 1] = y_axis
        # rot_matrix[0:3, 2] = z_axis

        # # Extract RPY from rotation matrix (XYZ convention)
        # pose_optimal_setpointO = tft.euler_from_matrix(rot_matrix, axes='sxyz')

        # method 3
        # Construct rotation that aligns X-axis with unit_direction
        # This aligns robot's local X-axis to unit_direction
        # Reference vector = [1, 0, 0] -> robot X axis in its own frame
        r_align, _ = Rotation.align_vectors([-unit_direction], [[1, 0, 0]])
        pose_optimal_setpointQ = r_align.as_quat()  # xyzw format
        pose_optimal_setpointO = r_align.as_euler('xyz')

        return pose_optimal_setpointL, pose_optimal_setpointO,  pose_optimal_setpointQ


    def compute_cmd_vel(self):
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
        

        optimal_setpointL, optimal_setpointO, optimal_setpointQ = self.compute_optimal_setpoint()
        self.optimal_pose = PoseStamped()
        self.optimal_pose.header.frame_id = "world"
        self.optimal_pose.pose.position.x = optimal_setpointL[0]
        self.optimal_pose.pose.position.y = optimal_setpointL[1]
        self.optimal_pose.pose.position.z = optimal_setpointL[2]
        self.optimal_pose.pose.orientation.x = optimal_setpointQ[0]
        self.optimal_pose.pose.orientation.y = optimal_setpointQ[1]
        self.optimal_pose.pose.orientation.z = optimal_setpointQ[2]
        self.optimal_pose.pose.orientation.w = optimal_setpointQ[3]

        self.errorLprev = self.errorL
        self.errorOprev = self.errorO

        self.errorL = optimal_setpointL - pose_currentL
        q_error = tft.quaternion_multiply(optimal_setpointQ, tft.quaternion_conjugate(pose_currentQ))
        self.errorO = np.array(tft.euler_from_quaternion(q_error))
        # self.errorO = optimal_setpointO - pose_currentO # this is wrong

        self.errorLsum = self.errorLsum + self.errorL
        self.errorOsum = self.errorOsum + self.errorO # this is also really wrong gotta do the whole PID thing later

        velocityL,velocityO = self.PID()

        # form cmd_vel message
        cmd_vel = Twist()
        cmd_vel.linear.x = velocityL[0]
        cmd_vel.linear.y = velocityL[1]
        cmd_vel.linear.z = velocityL[2]
        cmd_vel.angular.x = velocityO[0]
        cmd_vel.angular.y = velocityO[1]
        cmd_vel.angular.z = velocityO[2]

        return cmd_vel

    def PID(self): 
        velocityL = self.LINEAR_K* ((self.LINEAR_P_GAIN*self.errorL) + (self.LINEAR_I_GAIN*self.errorLsum*self.dt) + (self.LINEAR_D_GAIN*((self.errorL - self.errorLprev)/self.dt)))
        velocityO = self.ANGULAR_K* ((self.ANGULAR_P_GAIN*self.errorO) + (self.ANGULAR_I_GAIN*self.errorOsum*self.dt) + (self.ANGULAR_D_GAIN*((self.errorO - self.errorOprev)/self.dt)))
        return velocityL, velocityO

    def current_pose_cb(self,msg: PoseStamped):
        self.current_pose = msg

    def current_velocity_cb(self,msg: Twist):
        self.current_velocity = msg

    def filtered_grasp_pose_cb(self,msg: PoseStamped):
        with self.mutex : 
            self._now = time.time()
            self.pose_message_timestamp_queue.append(self._now)
            self.proxy_filtered_grasp_pose = msg
            
    def update_grasp_pose(self,event):
        with self.mutex : 
            self.pose_message_timestamp_queue = [t for t in self.pose_message_timestamp_queue if time.time() - t <= 1]
            if len(self.pose_message_timestamp_queue) < self.pose_setpoint_frequency_cutoff:
                self.filtered_grasp_pose = None
            else :
                self.filtered_grasp_pose = self.proxy_filtered_grasp_pose
        

    def load_parameters(self,file_path):
        with open(file_path, 'r') as file:
            try:
                parameters = yaml.safe_load(file)
                return parameters
            except yaml.YAMLError as exc:
                print(f"Error loading YAML file: {exc}")
                return None


if __name__ == "__main__":
    rospy.init_node("radial_tracker")
    radial_tracker = RadialTracker()
    rospy.spin()