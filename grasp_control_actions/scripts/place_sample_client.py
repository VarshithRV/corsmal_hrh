# write a an action client for the rests
import rospy
from grasp_control_actions.msg import PlaceMsgAction, PlaceMsgActionGoal, PlaceMsgActionFeedback, PlaceMsgActionResult, PlaceMsgGoal
import actionlib
import threading

class MpClass:
    def __init__(self):
        self.place_client = actionlib.SimpleActionClient("place_server", PlaceMsgAction)
        rospy.loginfo("waiting for server")
        self.place_client.wait_for_server()
        rospy.loginfo("server connected")


if __name__ == "__main__":
    rospy.init_node("motion_planning_client")
    mp = MpClass()
    rospy.loginfo("Sending pick and place goal")
    
    placeGoal = PlaceMsgGoal()
    mp.place_client.send_goal(placeGoal)
    result = mp.place_client.get_result()
    print("Result before completion : ",result)
    print("sleeping for 0.5s and then stopping")
    rospy.sleep(1)
    # mp.place_client.cancel_goal()
    # print("Tried to cancel the goal")
    mp.place_client.wait_for_result()
    result = mp.place_client.get_result()
    print("Rest result is: ",mp.place_client.get_result())