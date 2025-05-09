#!/usr/bin/env python
# this is a node that should host an action server that will place the object in hand to a default location or a location provided in the goal
# should be uninterruptible
# will have to load an unload joint_traj type controller, for simulation, use the pos_joint_traj_controller
# switch controllers between joint_group_pos_controller and pos_joint_traj_controller
# should initialize scene?

import moveit_commander.roscpp_initializer
import yaml
import rospy
import tf.transformations as tft
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Twist, Pose, TransformStamped
import time
import numpy as np
from threading import Lock
import math
import actionlib
from grasp_control_actions.msg import PlaceMsgAction, PlaceMsgActionResult, PlaceMsgGoal, PlaceMsgFeedback
import moveit_commander, moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
from controller_manager_msgs.srv import SwitchController,SwitchControllerRequest, SwitchControllerResponse
import sys
import quaternion
import copy
from math import pi, tau, dist, fabs, cos


class Place:
    def __init__(self):
        
        self.params = rospy.get_param("/place_blended_radius_as")
        self.common_params = rospy.get_param("/common_parameters")

        self.servo_topic = self.common_params["servo_topic"]
        self.servo_controller = self.common_params["servo_controller"]
        self.traj_controller = self.common_params["traj_controller"]

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
        self.go_back = self.params["go_back"]
        self.current_pose = None
        self.place_position = None
        self._waypoints = []

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

        rospy.loginfo("%s : Started the place node with parameters:"%rospy.get_name())
        for item in self.params:
            rospy.loginfo(f"{item} : {self.params[item]}")
        for item in self.common_params:
            rospy.loginfo(f"{item} : {self.common_params[item]}")
        
        self.start = rospy.Time.now()

        # create action server for Place
        self.place_action_server = actionlib.SimpleActionServer(
            "place_blended_radius_server", PlaceMsgAction, self.place_action_callback, auto_start=False
        )
        # Timer to update current_pose
        current_pose_update_timer = rospy.Timer(rospy.Duration(0.01),callback=self.update_current_pose)
        # publish error and velocity magnitude for debug
        self.linear_error_publisher =  rospy.Publisher("/linear_error",Float32,queue_size=1)
        # error publisher timer
        error_publisher_timer = rospy.Timer(rospy.Duration(0.01),callback=self.publish_error)
        # service proxy for switch controllers
        self.switch_controller = rospy.ServiceProxy("/controller_manager/switch_controller",SwitchController)

        # preempt registration
        self.place_action_server.register_preempt_callback(self.place_preempt_callback)
        self.place_action_server.start()
    
    def publish_error(self, event):
        if self.place_position is not None:
            current_pose = np.array([self.current_pose.pose.position.x,self.current_pose.pose.position.y,self.current_pose.pose.position.z],dtype=float)
            setpoint_pose = np.array([self.place_position.pose.position.x,self.place_position.pose.position.y,self.place_position.pose.position.z],dtype=float)
            self.linear_error_publisher.publish(Float32(data = np.linalg.norm(current_pose-setpoint_pose)))

    def execute_waypoints(self, waypoints):
        # rospy.loginfo("%s : Executing waypoints : %s", rospy.get_name(),waypoints)

        # plan a cartesian path
        # Need to check how to make this faster
        try : 
            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.005,  # eef_step
            )
            plan=self.move_group.retime_trajectory(self.move_group.get_current_state(),plan,velocity_scaling_factor = 1.0,algorithm="time_optimal_trajectory_generation")

        except Exception as e:
            rospy.logerr("%s : %s",rospy.get_name(),e)
            return False

        # display the plan
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        try : 
            self.move_group.execute(plan, wait=True)
            self.move_group.stop()
        except Exception as e:
            rospy.logerr("%s : %s",rospy.get_name(),e)
            return False

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
        rospy.loginfo("%s : Preempt requested, not supported for place",rospy.get_name())

    def is_close(self,array1, array2):
        return True if np.isclose(np.array(array1),np.array(array2), 0.01,0.01,False).all() else False

    def update_current_pose(self,event):
        self.current_pose = self.move_group.get_current_pose() # as a PoseStamped

    def place_action_callback(self,goal:PlaceMsgGoal):
        start = rospy.get_time()
        rospy.loginfo("%s : Action started",rospy.get_name())
        if self.switch_controller_to_moveit() : 
            pass
        else : 
            rospy.logerr("%s : Controller not switched from joint_group_pos_controller to pos_joint_traj_controller, aborting",rospy.get_name())
            self.place_action_server.set_aborted(PlaceMsgActionResult(result=False))
            return
        
        rospy.loginfo("%s : Starting place now",rospy.get_name())
        place_position = goal.place_position if not self.use_default_position else self.default_position
        self.place_position = place_position
        preplace_position = PoseStamped()
        _preplace_position = np.array([place_position.pose.position.x,place_position.pose.position.y,place_position.pose.position.z])
        _preplace_orientation = quaternion.from_float_array([place_position.pose.orientation.w, place_position.pose.orientation.x,place_position.pose.orientation.y,place_position.pose.orientation.z])
        _preplace_orientation_rotation = quaternion.from_float_array([self.preplace_transformation.pose.orientation.w,self.preplace_transformation.pose.orientation.x,self.preplace_transformation.pose.orientation.y,self.preplace_transformation.pose.orientation.z])
        _pure_position_quaternion = quaternion.from_float_array([0.0,self.preplace_transformation.pose.position.x,self.preplace_transformation.pose.position.y,self.preplace_transformation.pose.position.z])
        _pure_rotated_position_quaternion = _preplace_orientation_rotation * _pure_position_quaternion * _preplace_orientation_rotation.conjugate()
        _rotated_position = np.array([_pure_rotated_position_quaternion.x,_pure_rotated_position_quaternion.y,_pure_rotated_position_quaternion.z])
        _preplace_position = _preplace_position + _rotated_position

        preplace_position.header.frame_id = ""
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

        post_place_position2 = copy.deepcopy(post_place_position)
        post_place_position2.pose.position.z += 0.1

        ## need to change from here to 

        # preplace_position -> place
        self._waypoints.clear()
        # self._waypoints.append(copy.deepcopy(preplace_position.pose))
        self._waypoints.append(copy.deepcopy(place_position.pose))
        self._waypoints.append(copy.deepcopy(post_place_position.pose))
        self._waypoints.append(copy.deepcopy(post_place_position2.pose))

        
        if self.execute_waypoints(waypoints = self._waypoints):
            pass
        else:
            self._waypoints.clear()
        self._waypoints.clear()

        
        ### here
        
        finish = rospy.get_time()

        rospy.loginfo("%s : Time taken for place : %s",rospy.get_name(),(finish-start))
        self.place_position = None
        self.place_action_server.set_succeeded(PlaceMsgActionResult(result=True))
        return

if __name__ == "__main__":
    rospy.init_node("place_br",argv=sys.argv)
    place = Place()
    rospy.sleep(0.5)
    rospy.spin()