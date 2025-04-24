# write a an action client for the rests
import rospy
from grasp_control_actions.msg import RestAction, RestActionGoal, RestActionFeedback, RestActionResult
import actionlib
import threading

class MpClass:
    def __init__(self):
        self.rest_client = actionlib.SimpleActionClient("rest", RestAction)
        rospy.loginfo("waiting for server")
        self.rest_client.wait_for_server()
        rospy.loginfo("server connected")


if __name__ == "__main__":
    rospy.init_node("motion_planning_client")
    mp = MpClass()
    rospy.loginfo("Sending pick and place goal")
    
    restGoal = RestActionGoal()
    mp.rest_client.send_goal(restGoal)
    result = mp.rest_client.get_result()
    print("Result before completion : ",result)
    print("sleeping for 0.5s and then stopping")
    rospy.sleep(1)
    # mp.yoink_client.cancel_goal()
    # print("Tried to cancel the goal")
    mp.rest_client.wait_for_result()
    result = mp.rest_client.get_result()
    print("Rest result is: ",mp.rest_client.get_result())