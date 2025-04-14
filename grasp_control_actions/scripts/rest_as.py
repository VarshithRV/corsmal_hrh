#!/usr/bin/env python
# this is a node that should host an action server that will move the robot to a rest state
# should be interruptible
# will have to load an unload joint_traj type controller, for simulation, use the pos_joint_traj_controller
# switch controllers between joint_group_pos_controller and pos_joint_traj_controller
# should initialize scene

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
from grasp_control_actions.msg import RestAction, RestActionFeedback, RestActionResult, RestActionGoal, RestFeedback
import moveit_commander, moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
from controller_manager_msgs.srv import SwitchController,SwitchControllerRequest, SwitchControllerResponse
import sys
import copy
from math import pi, tau, dist, fabs, cos


class Rest:
    def __init__(self):
        
        self.params = rospy.get_param("/rest_as")

        self.base = self.params["rest_joint_state"]["base"]
        self.shoulder = self.params["rest_joint_state"]["shoulder"]
        self.elbow = self.params["rest_joint_state"]["elbow"]
        self.wrist1 = self.params["rest_joint_state"]["wrist1"]
        self.wrist2 = self.params["rest_joint_state"]["wrist2"]
        self.wrist3 = self.params["rest_joint_state"]["wrist3"]

        self.rest_joint_state = [self.base, self.shoulder, self.elbow, self.wrist1, self.wrist2, self.wrist3]
        self._waypoints = []

        rospy.loginfo("%s : Started the rest node with parameters:",rospy.get_name())
        for item in self.params:
            rospy.loginfo(f"{item} : {self.params[item]}")
        
        self.start = rospy.Time.now()

        # create action server for Rest
        self.rest_action_server = actionlib.SimpleActionServer(
            "rest", RestAction, self.rest_action_callback, auto_start=False
        )

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

        # preempt registration
        self.rest_action_server.register_preempt_callback(self.rest_preempt_callback)
        self.rest_action_server.start()

    def switch_controller_to_moveit(self):
        switch_controller_msg = SwitchControllerRequest()
        switch_controller_msg.start_controllers =  ["pos_joint_traj_controller"]
        switch_controller_msg.stop_controllers = ["joint_group_pos_controller"]
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
        switch_controller_msg.start_controllers = ["joint_group_pos_controller"]
        switch_controller_msg.stop_controllers = ["pos_joint_traj_controller"]
        switch_controller_msg.strictness = 2
        switch_controller_msg.start_asap = True
        try : 
            success = self.switch_controller.call(switch_controller_msg)
        except rospy.ServiceException as e:
            rospy.loginfo("%s : %s",rospy.get_name(),e)
        if success : 
            rospy.loginfo("%s : Controller switched to joint position group",rospy.get_name())
        return success

    def rest_preempt_callback(self):
        rospy.loginfo("%s : Preempt requested",rospy.get_name())

    def is_close(self,array1, array2):
        return True if np.isclose(np.array(array1),np.array(array2), 0.01,0.01,False).all() else False

    def rest_action_callback(self,goal:RestActionGoal):
        start = rospy.get_time()
        rospy.loginfo("%s : Action started",rospy.get_name())
        if self.switch_controller_to_moveit() : 
            pass
        else : 
            rospy.logerr("%s : Controller not switch to the right one, aborting",rospy.get_name())
            self.rest_action_server.set_aborted(RestActionResult(result=False))
            return
        rospy.loginfo("%s : Commanding robot to go to joint state : %s",rospy.get_name(),(str(self.rest_joint_state)))
        try : 
            self.move_group.go(self.rest_joint_state, wait=False)
        except Exception as e:
            rospy.loginfo("%s : Reaching joint state failed due to : %s",rospy.get_name(),e)
            self.rest_action_server.set_succeeded(RestActionResult(result=False))
        rate = rospy.Rate(30)
        while not self.is_close(self.rest_joint_state,self.move_group.get_current_joint_values()) and not self.rest_action_server.is_preempt_requested():
            rate.sleep()
        try :
            self.move_group.stop()
        except Exception as e:
            rospy.logerr("%s : Could not stop, exception : %s",rospy.get_name(),e)
            
        finish = rospy.get_time()
        
        rospy.loginfo("%s : Time taken for rest : %s",rospy.get_name(),(finish-start))
        self.rest_action_server.set_succeeded(RestActionResult(result=True))

if __name__ == "__main__":
    rospy.init_node("rest")
    rest = Rest()
    rospy.sleep(0.5)
    rospy.spin()