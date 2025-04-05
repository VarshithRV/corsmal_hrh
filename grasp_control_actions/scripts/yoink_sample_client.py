# write a an action client for the pick and place action server
import rospy
from grasp_control_actions.msg import YoinkAction, YoinkActionGoal, YoinkActionFeedback, YoinkActionResult
import actionlib

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
    mp.yoink_client.wait_for_result()
    print("Yoink result is: ",mp.yoink_client.get_result())