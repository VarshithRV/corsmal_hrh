#!/usr/bin/env python
# server that places using velocity control

import yaml
import rospy
import tf.transformations as tft
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, TransformStamped
import time
import numpy as np
from threading import Lock
import math
import actionlib
from grasp_control_actions.msg import PlaceVelAction, PlaceVelActionFeedback, PlaceVelActionResult, PlaceVelActionGoal, PlaceVelFeedback
from controller_manager_msgs.srv import SwitchController,SwitchControllerRequest, SwitchControllerResponse
import moveit_commander, moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import quaternion
import sys
import copy


class Place:
    def __init__(self):
        
        self.params = rospy.get_param("/place_traj_vel_as")
        self.common_params = rospy.get_param("/common_parameters")

        self.servo_topic = self.common_params["servo_topic"]
        self.servo_controller = self.common_params["servo_controller"]
        self.traj_controller = self.common_params["traj_controller"]
        
        self.LINEAR_P_GAIN = self.params["gains"]["linear"]["p"]
        self.LINEAR_I_GAIN = self.params["gains"]["linear"]["i"]
        self.LINEAR_D_GAIN = self.params["gains"]["linear"]["d"]
        self.LINEAR_K = self.params["gains"]["linear"]["k"]
        self.ANGULAR_P_GAIN = self.params["gains"]["angular"]["p"]
        self.ANGULAR_I_GAIN = self.params["gains"]["angular"]["i"]
        self.ANGULAR_D_GAIN = self.params["gains"]["angular"]["d"]
        self.ANGULAR_K = self.params["gains"]["angular"]["k"]

        self.cmd_publish_frequency = self.params["cmd_publish_frequency"]
        self.alpha = self.params["alpha"]

        self.max_linear_velocity = self.params["max_linear_velocity"]
        self.max_linear_acceleration = self.params["max_linear_acceleration"]
        self.max_angular_velocity = self.params["max_angular_velocity"]
        self.max_angular_acceleration = self.params["max_angular_acceleration"]

        self.pose_setpoint_frequency_cutoff = self.params["pose_setpoint_frequency_cuttoff"]
        self.linear_stop_threshold = self.params["linear_stop_threshold"]
        self.angular_stop_threshold = self.params["angular_stop_threshold"]
        self.pre_grasp_transform = self.params["pre_grasp_transform"]
        self.input_stream_timeout = self.params["input_stream_timeout"]

        self.go_back = self.params["go_back"]
        self.acceleration = self.params["acceleration"]

        self.default_position = PoseStamped()
        self.default_position.header.frame_id = "world"
        self.default_position.pose.position.x =self.params["default_position"]["position"]["x"]
        self.default_position.pose.position.y =self.params["default_position"]["position"]["y"]
        self.default_position.pose.position.z =self.params["default_position"]["position"]["z"]
        self.default_position.pose.orientation.x =self.params["default_position"]["orientation"]["x"]
        self.default_position.pose.orientation.y =self.params["default_position"]["orientation"]["y"]
        self.default_position.pose.orientation.z =self.params["default_position"]["orientation"]["z"]
        self.default_position.pose.orientation.w =self.params["default_position"]["orientation"]["w"]

        self.preplace_transformation = PoseStamped()
        self.preplace_transformation.header.frame_id = "world"
        self.preplace_transformation.pose.position.x =self.params["preplace_transformation"]["position"]["x"]
        self.preplace_transformation.pose.position.y =self.params["preplace_transformation"]["position"]["y"]
        self.preplace_transformation.pose.position.z =self.params["preplace_transformation"]["position"]["z"]
        self.preplace_transformation.pose.orientation.x =self.params["preplace_transformation"]["orientation"]["x"]
        self.preplace_transformation.pose.orientation.y =self.params["preplace_transformation"]["orientation"]["y"]
        self.preplace_transformation.pose.orientation.z =self.params["preplace_transformation"]["orientation"]["z"]
        self.preplace_transformation.pose.orientation.w =self.params["preplace_transformation"]["orientation"]["w"]

        self.use_default_position = self.params["use_default_position"]

        self.dt = 1/self.cmd_publish_frequency
        self.errorL = np.zeros((3,),dtype=float)
        self.errorO = np.array([0,0,0],dtype=float)
        self.errorLsum = np.zeros((3,),dtype=float)
        self.errorOsum = np.array([0,0,0],dtype=float)
        self.errorLprev = np.zeros((3,),dtype=float)
        self.errorOprev = np.array([0,0,0],dtype=float)
        self.feedback = PlaceVelFeedback()

        rospy.loginfo("%s : Started the place node with parameters:",rospy.get_name())
        for item in self.params:
            rospy.loginfo(f"{item} : {self.params[item]}")
        for item in self.common_params:
            rospy.loginfo(f"{item} : {self.common_params[item]}")
            
        self.start = rospy.Time.now()
        
        self.current_pose = None
        self.setpoint_velocity = None
        self.filtered_grasp_pose = None
        self.proxy_filtered_grasp_pose = None
        self.input_stream_status = False
        self.last_message_time = None
        self.prev_last_message_time = None
        self.linear_velocity = 0.0
        self.angular_velocity  = 0.0
        self.linear_error = 0.0
        self.angular_error = 0.0
        self.filtered_linear_cmd_vel = np.array([0,0,0],dtype = float)
        self.filtered_angular_cmd_vel = np.array([0,0,0],dtype = float)

        # error gradient calculation tools
        self.pid_error_gradient = np.zeros((3,),dtype =float)
        self.pid_error_gradient_alpha = 0.9
        self.time_stamped_errorL = {}
        self.time_stamped_errorL_list = []

        self.mutex = Lock()
        self._now = None
        self.optimal_pose = None

        # moveit stuff
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        # service proxy for switch controllers
        self.switch_controller = rospy.ServiceProxy("/controller_manager/switch_controller",SwitchController)
        # simulation interface
        self.setpoint_velocity_pub = rospy.Publisher(self.servo_topic,TwistStamped,queue_size=1)
        self.optimal_pose_pub =  rospy.Publisher("/optimal_pose",PoseStamped,queue_size=1)
        # publish error and velocity magnitude for debug
        self.linear_error_publisher =  rospy.Publisher("/linear_error",Float32,queue_size=1)
        self.angular_error_publisher = rospy.Publisher("/angular_error",Float32,queue_size=1)
        self.linear_velocity_publisher = rospy.Publisher("/linear_velocity",Float32,queue_size=1)
        self.angular_velocity_publisher = rospy.Publisher("/angular_velocity",Float32,queue_size=1)
        self.pid_gradient_publisher = rospy.Publisher("/pid_error_gradient",Float32,queue_size = 1)

        # filtered grasp pose connection
        self.filtered_grasp_pose_sub = rospy.Subscriber("/filtered_grasp_pose",PoseStamped,callback=self.filtered_grasp_pose_cb)

        # Timer to update current_pose
        current_pose_update_timer = rospy.Timer(rospy.Duration(0.01),callback=self.update_current_pose)

        # Timer to update input stream status
        input_stream_status_update_timer = rospy.Timer(rospy.Duration(0.01),callback=self.is_input_stream_active)

        # Timer to update the filtered grasp pose
        filtered_grasp_pose_update_timer = rospy.Timer(rospy.Duration(0.01),callback=self.update_grasp_pose)

        # Timer to publish the optimal pose
        op_pose_publisher_timer = rospy.Timer(rospy.Duration(0.01),callback=self.publish_optimal_pose)

        # Timer to publish the error velocity
        error_velocity_publisher_timer = rospy.Timer(rospy.Duration(0.01),callback=self.publish_error_velocity)

        # Timer to log input stream diagnostics
        input_stream_diagnostics_timer = rospy.Timer(rospy.Duration(1), callback=self.filtered_grasp_pose_diagnostics)
        
        # update parameters timer
        update_parameters_timer = rospy.Timer(rospy.Duration(3),callback=self.update_parameters)

        # calculate pid error gradient timer
        calc_pid_error_gradient_timer = rospy.Timer(rospy.Duration(0.01),callback=self.calc_pid_error_gradient)

        self._qsum  = np.zeros((4,1),dtype=float)

        # create action server for Place
        self.place_action_server = actionlib.SimpleActionServer(
            "place_traj_vel", PlaceVelAction, self.place_action_callback, auto_start=False
        )
        self.place_action_server.register_preempt_callback(self.place_preempt_callback)
        self.place_action_server.start()
    
    def filter_command_vel(self,vel:TwistStamped):
        self.filtered_linear_cmd_vel = self.alpha*(self.filtered_linear_cmd_vel) + (1-self.alpha)*(np.array([vel.twist.linear.x,vel.twist.linear.y,vel.twist.linear.z]))
        vel_out = TwistStamped()
        vel_out.header = vel.header
        vel_out.twist.angular = vel.twist.angular
        vel_out.twist.linear.x = self.filtered_linear_cmd_vel[0]
        vel_out.twist.linear.y = self.filtered_linear_cmd_vel[1]
        vel_out.twist.linear.z = self.filtered_linear_cmd_vel[2]
        return vel_out

    def calc_pid_error_gradient(self,event):
        # modify self.pid_error_gradient here(sliding window + IIR filter)
        # use self.pid_error_gradient_alpha for IIR filter
        # use the timestamped list : self.time_stamped_errorL_list
        sliding_window_size = 5
        error_error = np.zeros((3,),dtype=float)
        delta_t = 0.01 # non zero to avoid divide by zero error
        if len(self.time_stamped_errorL_list) <= 1:
            error_error = 0.0
        elif len(self.time_stamped_errorL_list) < 5 and len(self.time_stamped_errorL_list) > 1:
            error_error = self.time_stamped_errorL_list[-1]["error"] - self.time_stamped_errorL_list[0]["error"]
            delta_t = self.time_stamped_errorL_list[-1]["stamp"] - self.time_stamped_errorL_list[0]["stamp"]
        else : 
            error_error = self.time_stamped_errorL_list[-1]["error"] - self.time_stamped_errorL_list[-1-sliding_window_size+1]["error"]
            delta_t = self.time_stamped_errorL_list[-1]["stamp"] - self.time_stamped_errorL_list[-1-sliding_window_size+1]["stamp"]
        gradient = error_error/delta_t
        self.pid_error_gradient = self.pid_error_gradient_alpha*self.pid_error_gradient + (1-self.pid_error_gradient_alpha)*gradient
        self.pid_gradient_publisher.publish(Float32(data = np.linalg.norm(self.pid_error_gradient)))

    def update_parameters(self,event):
        self.params = rospy.get_param("/place_vel_as")
        self.common_params = rospy.get_param("/common_parameters")

        self.servo_topic = self.common_params["servo_topic"]
        self.servo_controller = self.common_params["servo_controller"]
        self.traj_controller = self.common_params["traj_controller"]
        
        self.LINEAR_P_GAIN = self.params["gains"]["linear"]["p"]
        self.LINEAR_I_GAIN = self.params["gains"]["linear"]["i"]
        self.LINEAR_D_GAIN = self.params["gains"]["linear"]["d"]
        self.LINEAR_K = self.params["gains"]["linear"]["k"]
        self.ANGULAR_P_GAIN = self.params["gains"]["angular"]["p"]
        self.ANGULAR_I_GAIN = self.params["gains"]["angular"]["i"]
        self.ANGULAR_D_GAIN = self.params["gains"]["angular"]["d"]
        self.ANGULAR_K = self.params["gains"]["angular"]["k"]

        self.cmd_publish_frequency = self.params["cmd_publish_frequency"]
        self.alpha = self.params["alpha"]

        self.max_linear_velocity = self.params["max_linear_velocity"]
        self.max_linear_acceleration = self.params["max_linear_acceleration"]
        self.max_angular_velocity = self.params["max_angular_velocity"]
        self.max_angular_acceleration = self.params["max_angular_acceleration"]

        self.pose_setpoint_frequency_cutoff = self.params["pose_setpoint_frequency_cuttoff"]
        self.linear_stop_threshold = self.params["linear_stop_threshold"]
        self.angular_stop_threshold = self.params["angular_stop_threshold"]
        self.pre_grasp_transform = self.params["pre_grasp_transform"]

        self.go_back = self.params["go_back"]

        self.default_position = PoseStamped()
        self.default_position.header.frame_id = "world"
        self.default_position.pose.position.x =self.params["default_position"]["position"]["x"]
        self.default_position.pose.position.y =self.params["default_position"]["position"]["y"]
        self.default_position.pose.position.z =self.params["default_position"]["position"]["z"]
        self.default_position.pose.orientation.x =self.params["default_position"]["orientation"]["x"]
        self.default_position.pose.orientation.y =self.params["default_position"]["orientation"]["y"]
        self.default_position.pose.orientation.z =self.params["default_position"]["orientation"]["z"]
        self.default_position.pose.orientation.w =self.params["default_position"]["orientation"]["w"]

        self.preplace_transformation = PoseStamped()
        self.preplace_transformation.header.frame_id = "world"
        self.preplace_transformation.pose.position.x =self.params["preplace_transformation"]["position"]["x"]
        self.preplace_transformation.pose.position.y =self.params["preplace_transformation"]["position"]["y"]
        self.preplace_transformation.pose.position.z =self.params["preplace_transformation"]["position"]["z"]
        self.preplace_transformation.pose.orientation.x =self.params["preplace_transformation"]["orientation"]["x"]
        self.preplace_transformation.pose.orientation.y =self.params["preplace_transformation"]["orientation"]["y"]
        self.preplace_transformation.pose.orientation.z =self.params["preplace_transformation"]["orientation"]["z"]
        self.preplace_transformation.pose.orientation.w =self.params["preplace_transformation"]["orientation"]["w"]

        self.use_default_position = self.params["use_default_position"]

    def update_current_pose(self,event):
        self.current_pose = self.move_group.get_current_pose() # as a PoseStamped

    def switch_controller_to_moveit(self):
        switch_controller_msg = SwitchControllerRequest()
        switch_controller_msg.start_controllers =  [self.traj_controller]
        switch_controller_msg.stop_controllers = [self.servo_controller]
        switch_controller_msg.strictness = 2
        switch_controller_msg.start_asap = True
        switch_controller_msg.timeout = 0.0
        try : 
            success = self.switch_controller.call(switch_controller_msg)
        except rospy.ServiceException as e:
            rospy.loginfo("%s : %s",rospy.get_name(),e)
        if success : 
            rospy.loginfo("%s : Controller switched to joint position trajectory",rospy.get_name())
        return success

    def switch_controller_to_servo(self):
        switch_controller_msg = SwitchControllerRequest()
        switch_controller_msg.start_controllers = [self.servo_controller]
        switch_controller_msg.stop_controllers = [self.traj_controller]
        switch_controller_msg.strictness = 2
        switch_controller_msg.start_asap = True
        try : 
            success = self.switch_controller.call(switch_controller_msg)
        except rospy.ServiceException as e:
            rospy.loginfo("%s : %s",rospy.get_name(),e)
        if success : 
            rospy.loginfo("%s : Controller switched to joint position group",rospy.get_name())
        return success

    def place_preempt_callback(self):
        rospy.loginfo("%s: Preempt requested",rospy.get_name())
        # reset all error variables
        self.errorLprev = np.zeros((3,),dtype=float)
        self.errorLsum = np.zeros((3,),dtype=float)
        self.errorOprev = np.zeros((4,),dtype=float)
        self.errorOsum = np.zeros((4,),dtype=float)
        self.pid_error_gradient = np.zeros((3,),dtype=float)
        self.time_stamped_errorL = {}
        self.time_stamped_errorL_list = []

    def place_action_callback(self,goal:PlaceVelActionGoal):
        start = rospy.get_time()
        rospy.loginfo("%s : Action started",rospy.get_name())
        
        if self.switch_controller_to_servo() : 
            pass
        else : 
            rospy.logerr("%s : Controller not switched from pos_joint_traj_controller to joint_group_pos_controller , aborting",rospy.get_name())
            self.place_action_server.set_aborted(PlaceVelActionResult(result=False))
            return
        
        place_position = goal.place_position if not self.use_default_position else self.default_position
        preplace_position = PoseStamped()
        _preplace_position = np.array([place_position.pose.position.x,place_position.pose.position.y,place_position.pose.position.z])
        _preplace_orientation = quaternion.from_float_array([place_position.pose.orientation.w, place_position.pose.orientation.x,place_position.pose.orientation.y,place_position.pose.orientation.z])
        _preplace_orientation_rotation = quaternion.from_float_array([self.preplace_transformation.pose.orientation.w,self.preplace_transformation.pose.orientation.x,self.preplace_transformation.pose.orientation.y,self.preplace_transformation.pose.orientation.z])
        _pure_position_quaternion = quaternion.from_float_array([0.0,self.preplace_transformation.pose.position.x,self.preplace_transformation.pose.position.y,self.preplace_transformation.pose.position.z])
        _pure_rotated_position_quaternion = _preplace_orientation_rotation * _pure_position_quaternion * _preplace_orientation_rotation.conjugate()
        _rotated_position = np.array([_pure_rotated_position_quaternion.x,_pure_rotated_position_quaternion.y,_pure_rotated_position_quaternion.z])
        _preplace_position = _preplace_position + _rotated_position

        preplace_position.header.frame_id = "world"
        preplace_position.pose.position.x,preplace_position.pose.position.y,preplace_position.pose.position.z = _preplace_position[0],_preplace_position[1],_preplace_position[2]
        preplace_position.pose.orientation.x,preplace_position.pose.orientation.y,preplace_position.pose.orientation.z,preplace_position.pose.orientation.w = _preplace_orientation.x,_preplace_orientation.y,_preplace_orientation.z,_preplace_orientation.w
        
        post_place_position = PoseStamped()
        post_place_position.pose.position.x = copy.deepcopy(place_position.pose.position.x - self.go_back)
        post_place_position.pose.position.y = copy.deepcopy(place_position.pose.position.y)
        post_place_position.pose.position.z = copy.deepcopy(place_position.pose.position.z)
        post_place_position.pose.orientation.w = copy.deepcopy(place_position.pose.orientation.w)
        post_place_position.pose.orientation.x = copy.deepcopy(place_position.pose.orientation.x)
        post_place_position.pose.orientation.y = copy.deepcopy(place_position.pose.orientation.y)
        post_place_position.pose.orientation.z = copy.deepcopy(place_position.pose.orientation.z)
        
        ############need to change stuff here################
        # grasp_result1 = self.goto_pose(preplace_position)
        grasp_result2 = self.goto_pose(place_position)
        grasp_result2 = self.goto_pose(post_place_position)
        result = PlaceVelActionResult()
        # result.result = grasp_result1 and grasp_result2
        result.result = grasp_result2
        finish = rospy.get_time()
        rospy.loginfo("%s : Time taken for place : %s",rospy.get_name(),(finish-start))
        self.place_action_server.set_succeeded(result=result)

    def publish_error_velocity(self,event):
        self.linear_error_publisher.publish(Float32(self.linear_error))
        self.angular_error_publisher.publish(Float32(self.angular_error))
        self.linear_velocity_publisher.publish(Float32(self.linear_velocity))
        self.angular_velocity_publisher.publish(Float32(self.angular_velocity))

    # goto a fixed PoseStamped
    # what are the responsibilitis of goto_pose
    # 1. initialize all error variables
    # 2. get optimal pose and current pose, all wrt base_link
    # 3. know when to publish null velocity
    # 4. publish linear and angular error as feedback
    # 5. The command velocity published is a TwistStamped with frame_id base_link
    def goto_pose(self, pose:PoseStamped):
        if pose is None :
            rospy.logerr("%s : improper pose received",rospy.get_name())
            return False

        # reset all error variables
        self.errorLprev = np.zeros((3,),dtype=float)
        self.errorLsum = np.zeros((3,),dtype=float)
        self.errorOprev = np.zeros((4,),dtype=float)
        self.errorOsum = np.zeros((4,),dtype=float)
        self.pid_error_gradient = np.zeros((3,),dtype=float)
        self.time_stamped_errorL = {}
        self.time_stamped_errorL_list = []
        
        optimal_poseL = np.array([pose.pose.position.x,pose.pose.position.y,pose.pose.position.z])
        optimal_poseQ = np.array([pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w])
        current_poseL = np.array([self.current_pose.pose.position.x,self.current_pose.pose.position.y,self.current_pose.pose.position.z])
        current_poseQ = np.array([self.current_pose.pose.orientation.x,self.current_pose.pose.orientation.y,self.current_pose.pose.orientation.z,self.current_pose.pose.orientation.w])

        unit_displacement_vector = (optimal_poseL - current_poseL)/np.linalg.norm(optimal_poseL - current_poseL)
        estimated_time_to_complete = math.sqrt(16*np.linalg.norm(optimal_poseL - current_poseL)/(3*self.acceleration))
        
        linear_velocity = np.zeros((3,),dtype=float)

        rate = rospy.Rate(self.cmd_publish_frequency)
        start = rospy.get_time()
        while not rospy.is_shutdown() :
            
            if not self.place_action_server.is_preempt_requested():
                optimal_poseL = np.array([pose.pose.position.x,pose.pose.position.y,pose.pose.position.z])
                optimal_poseQ = np.array([pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w])

                current_poseL = np.array([self.current_pose.pose.position.x,self.current_pose.pose.position.y,self.current_pose.pose.position.z])
                current_poseQ = np.array([self.current_pose.pose.orientation.x,self.current_pose.pose.orientation.y,
                                          self.current_pose.pose.orientation.z,self.current_pose.pose.orientation.w])

                if abs(np.linalg.norm(optimal_poseL-current_poseL)) < self.linear_stop_threshold and abs(np.linalg.norm(optimal_poseQ-current_poseQ))<self.angular_stop_threshold : 
                    cmd_vel = TwistStamped()
                    cmd_vel.header.stamp = rospy.Time.now()
                    cmd_vel.header.frame_id = "world"
                    rospy.loginfo("%s : Reached pose",rospy.get_name())

                    start_to_stop_timer = rospy.get_time()
                    
                    # publish zero velocity for good measure
                    while not rospy.is_shutdown():
                        if rospy.get_time() - start_to_stop_timer > 0.15:
                            break
                        self.setpoint_velocity_pub.publish(cmd_vel)
                        rate.sleep()

                    # reset all error variables
                    self.errorLprev = np.zeros((3,),dtype=float)
                    self.errorLsum = np.zeros((3,),dtype=float)
                    self.errorOprev = np.zeros((4,),dtype=float)
                    self.errorOsum = np.zeros((4,),dtype=float)
                    self.pid_error_gradient = np.zeros((3,),dtype=float)
                    self.time_stamped_errorL = {}
                    self.time_stamped_errorL_list = []
                    return True

                self.linear_error = np.linalg.norm(optimal_poseL-current_poseL)
                self.angular_error = np.linalg.norm(optimal_poseQ-current_poseQ)

                cmd_vel = self.compute_cmd_vel(optimal_setpointL=optimal_poseL,optimal_setpointQ=optimal_poseQ)
                # this needs to be changed in cmd_vel, then we are done, angular velocity can remain the same
                cmd_vel.twist.linear.x = linear_velocity[0]
                cmd_vel.twist.linear.y = linear_velocity[1]
                cmd_vel.twist.linear.z = linear_velocity[2]
                dt = 1/self.cmd_publish_frequency
                a = self.acceleration
                acceleration_vector = a*unit_displacement_vector
                print("########################")
                print(unit_displacement_vector)
                print(acceleration_vector)
                print(current_poseL)
                print(optimal_poseL)
                print(linear_velocity)
                print("########################")
                elapsed = rospy.get_time() - start
                if elapsed <= estimated_time_to_complete/4: # acceleration
                    linear_velocity += acceleration_vector*dt
                elif estimated_time_to_complete/4 < elapsed and elapsed <= (3*estimated_time_to_complete)/4: # constant velocity
                    linear_velocity = linear_velocity
                elif elapsed >= (3*estimated_time_to_complete)/4: # retardation
                    linear_velocity -= acceleration_vector*dt
                else :
                    linear_velocity = np.zeros((3,),dtype=float)
                
                if np.linalg.norm(linear_velocity) > 1.0:
                        linear_velocity /= np.linalg.norm(linear_velocity)
                #############################################################################################

                self.setpoint_velocity = cmd_vel
                self.setpoint_velocity.header.stamp = rospy.Time.now()
                self.setpoint_velocity_pub.publish(cmd_vel)

                # need to publish feedback here for the action
                self.feedback.linear_error.data = self.linear_error
                self.feedback.linear_velocity.data = self.linear_velocity
                self.feedback.angular_velocity.data = self.angular_velocity
                self.feedback.angular_error.data = self.angular_error
                self.place_action_server.publish_feedback(self.feedback)

                rate.sleep()
            else : 
                rospy.loginfo("%s : Preempted requested while in pre grasp",rospy.get_name())
                return False
     
    # Computes the velocity to command
    def compute_cmd_vel(self,optimal_setpointL,optimal_setpointQ):
        # check if current_pose
        if self.current_pose is None:
            # rospy.logerr("Current pose or velociy is not received")
            vel= TwistStamped()
            vel.header.frame_id = "base_link"
            return vel
        
        pose_current = self.current_pose
        pose_currentL = np.array([pose_current.pose.position.x,pose_current.pose.position.y,pose_current.pose.position.z])
        pose_currentO = np.array(tft.euler_from_quaternion([pose_current.pose.orientation.w,pose_current.pose.orientation.x,
                                                    pose_current.pose.orientation.y,pose_current.pose.orientation.z]))
        pose_currentQ = np.array([pose_current.pose.orientation.x,pose_current.pose.orientation.y,
                                    pose_current.pose.orientation.z,pose_current.pose.orientation.w])

        self.optimal_pose = PoseStamped()
        self.optimal_pose.header.frame_id = "base_link"
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
        self.time_stamped_errorL = {"stamp":rospy.get_time(),"error":self.errorL}
        self.time_stamped_errorL_list.append(self.time_stamped_errorL)
        
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
        cmd_vel = TwistStamped()
        cmd_vel.header.frame_id = "base_link"
        cmd_vel.twist.linear.x = velocityL[0]
        cmd_vel.twist.linear.y = velocityL[1]
        cmd_vel.twist.linear.z = velocityL[2]
        cmd_vel.twist.angular.x = velocityO[0]
        cmd_vel.twist.angular.y = velocityO[1]
        cmd_vel.twist.angular.z = velocityO[2]
        return cmd_vel

    # publish optimal pose to debug
    def publish_optimal_pose(self,event):
        if self.optimal_pose is not None :
            self.optimal_pose_pub.publish(self.optimal_pose)

    # compute velocityL and velocityO from errorL and errorO
    def PID(self,errorL,errorLsum,errorLdiff,errorO,errorOsum,errorOdiff): 
        velocityL = self.LINEAR_K* ((self.LINEAR_P_GAIN*errorL) + (self.LINEAR_I_GAIN*errorLsum*self.dt) + (self.LINEAR_D_GAIN*(self.pid_error_gradient)))
        velocityO = self.ANGULAR_K* ((self.ANGULAR_P_GAIN*errorO) + (self.ANGULAR_I_GAIN*errorOsum*self.dt) + (self.ANGULAR_D_GAIN*(errorOdiff/self.dt)))
        return velocityL, velocityO

    def filtered_grasp_pose_cb(self,msg: PoseStamped):
        with self.mutex :
            self.prev_last_message_time = self.last_message_time
            self.last_message_time = rospy.get_time()
            self.proxy_filtered_grasp_pose = msg
    
    def is_input_stream_active(self, event):
        if self.prev_last_message_time is not None: # check if atleast two messages have been received
            if rospy.get_time() - self.last_message_time <=self.input_stream_timeout: # check if the last message received is with a small amount of time > 0.01s
                if self.last_message_time - self.prev_last_message_time <= self.input_stream_timeout: # the messages have greater frequency
                    self.input_stream_status=True
                else :
                    self.input_stream_status=False
            else:
                self.input_stream_status=False
        else:
            self.input_stream_status=False
        
    def filtered_grasp_pose_diagnostics(self, event):
        if not self.input_stream_status:
            # rospy.logwarn("%s : Input Stream error, message not received within timeout"%rospy.get_name())
            pass
    
    # update grasp pose if stream is active
    def update_grasp_pose(self,event):
        if self.input_stream_timeout : 
            self.filtered_grasp_pose = self.proxy_filtered_grasp_pose
        else : 
            self.filtered_grasp_pose = None


if __name__ == "__main__":
    rospy.init_node("place_traj_vel")
    place = Place()
    rospy.sleep(0.5)
    rospy.spin()