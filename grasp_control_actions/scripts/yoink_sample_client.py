# write a an action client for the pick and place action server
'''
required capabilities : 
1. async start
2. wait for result
2. preempt 
3. feeback monitoring : /yoink/feedback
'''
import rospy
from grasp_control_actions.msg import YoinkAction, YoinkActionGoal, YoinkActionFeedback, YoinkActionResult
import actionlib
import threading

class MpClass:
    def __init__(self):
        self.yoink_client = actionlib.SimpleActionClient("yoink", YoinkAction)
        rospy.loginfo("waiting for server")
        self.yoink_client.wait_for_server()
        rospy.loginfo("server connected")


if __name__ == "__main__":
    rospy.init_node("motion_planning_client")
    mp = MpClass()
    rospy.loginfo("Sending pick and place goal")
    
    yoinkGoal = YoinkActionGoal()
    mp.yoink_client.send_goal(yoinkGoal)
    result = mp.yoink_client.get_result()
    print("Result before completion : ",result)
    print("sleeping for 0.5s and then stopping")
    rospy.sleep(2)
    # mp.yoink_client.cancel_goal()
    # print("Tried to cancel the goal")
    # mp.yoink_client.wait_for_result()
    result = mp.yoink_client.get_result()
    print("Yoink result is: ",mp.yoink_client.get_result())