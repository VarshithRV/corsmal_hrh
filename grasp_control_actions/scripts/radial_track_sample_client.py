# write a an action client for the pick and place action server
'''
required capabilities : 
1. async start
2. wait for result
2. preempt 
3. feeback monitoring : /radial_track/feedback
'''
import rospy
from grasp_control_actions.msg import RadialTrackingAction, RadialTrackingGoal, RadialTrackingActionGoal, RadialTrackingActionFeedback, RadialTrackingActionResult
import actionlib
import threading

class MpClass:
    def __init__(self):
        self.radial_tracking_client = actionlib.SimpleActionClient("radial_track", RadialTrackingAction)
        rospy.loginfo("waiting for server")
        self.radial_tracking_client.wait_for_server()
        rospy.loginfo("server connected")


if __name__ == "__main__":
    rospy.init_node("motion_planning_client")
    mp = MpClass()
    rospy.loginfo("Sending pick and place goal")
    radial_trackGoal = RadialTrackingGoal()
    radial_trackGoal.timeout.data = 0.0 # set 0 to run indefinitely, otherwise give a positive value
    mp.radial_tracking_client.send_goal(radial_trackGoal)
    result = mp.radial_tracking_client.get_result()
    print("Result before completion : ",result)
    print("sleeping for 0.5s and then stopping")
    rospy.sleep(2)
    # print("Tried to cancel the goal")
    mp.radial_tracking_client.cancel_goal()
    # mp.yoink_client.wait_for_result()
    result = mp.radial_tracking_client.get_result()
    print("Radial Tracking result is: ",mp.radial_tracking_client.get_result())