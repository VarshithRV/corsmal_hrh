# write a an action client for the rests
import rospy
from grasp_control_actions.msg import PlaceMsgAction, PlaceMsgActionGoal, PlaceMsgActionFeedback, PlaceMsgActionResult, PlaceMsgGoal
from grasp_control_actions.msg import PlaceVelAction, PlaceVelActionGoal, PlaceVelActionFeedback, PlaceVelActionResult, PlaceVelGoal
from grasp_control_actions.msg import RadialTrackingAction, RadialTrackingGoal, RadialTrackingActionGoal, RadialTrackingActionFeedback, RadialTrackingActionResult
from grasp_control_actions.msg import YoinkAction, YoinkActionGoal, YoinkActionFeedback, YoinkActionResult
from grasp_control_actions.msg import RestAction, RestActionGoal, RestActionFeedback, RestActionResult
import actionlib
import threading

class MpClass:
    def __init__(self):
        rospy.loginfo("%s Started client, connecting to action servers", rospy.get_name())
        self.place_client = actionlib.SimpleActionClient("place_server", PlaceMsgAction)
        self.place_vel_client = actionlib.SimpleActionClient("place_vel", PlaceVelAction)
        self.rest_client = actionlib.SimpleActionClient("rest", RestAction)
        self.radial_tracking_client = actionlib.SimpleActionClient("radial_track", RadialTrackingAction)
        self.yoink_client = actionlib.SimpleActionClient("yoink", YoinkAction)
        rospy.loginfo("%s Waiting for clients", rospy.get_name())
        client_connection1 = self.yoink_client.wait_for_server(timeout=rospy.Duration(secs=3))
        client_connection2 = self.radial_tracking_client.wait_for_server(timeout=rospy.Duration(secs=3))
        client_connection3 = self.rest_client.wait_for_server(timeout=rospy.Duration(secs=3))
        client_connection4 = self.place_client.wait_for_server(timeout=rospy.Duration(secs=3))
        client_connection5 = self.place_vel_client.wait_for_server(timeout=rospy.Duration(secs=3))
        if not (client_connection1 and client_connection2 and client_connection3 and client_connection4 and client_connection5):
            rospy.logwarn("%s Some clients are not connected!!",rospy.get_name())
        else : 
            rospy.loginfo("%s : All servers connected",rospy.get_name())


if __name__ == "__main__":
    rospy.init_node("motion_planning_client")
    mp = MpClass()
    
    start = rospy.get_time()

    # radial track for a few seconds
    rospy.loginfo("%s : radial tracking",rospy.get_name())
    radial_trackGoal = RadialTrackingGoal()
    radial_trackGoal.timeout.data = 0.8
    mp.radial_tracking_client.send_goal(radial_trackGoal)
    mp.radial_tracking_client.wait_for_result()
    result = mp.radial_tracking_client.get_result()
    rospy.loginfo("%s : radial tracking result : %s",rospy.get_name(), result)

    # yoink
    rospy.loginfo("%s : yoink",rospy.get_name())
    yoinkGoal = YoinkActionGoal()
    mp.yoink_client.send_goal(yoinkGoal)
    mp.yoink_client.wait_for_result()
    result = mp.yoink_client.get_result()
    rospy.loginfo("%s : yoink result : %s",rospy.get_name(), result)

    # place
    rospy.loginfo("%s : place",rospy.get_name())
    placeGoal = PlaceMsgGoal()
    mp.place_client.send_goal(placeGoal)
    # mp.place_client.cancel_goal()
    mp.place_client.wait_for_result()
    result = mp.place_client.get_result()
    rospy.loginfo("%s : place result : %s",rospy.get_name(), result)
    
    # # place vel
    # rospy.loginfo("%s : place",rospy.get_name())
    # placeGoal = PlaceVelGoal()
    # mp.place_vel_client.send_goal(placeGoal)
    # # mp.place_client.cancel_goal()
    # mp.place_vel_client.wait_for_result()
    # result = mp.place_vel_client.get_result()
    # rospy.loginfo("%s : place result : %s",rospy.get_name(), result)

    finish = rospy.get_time()
    # report
    rospy.loginfo("%s : time taken to finish is %s",rospy.get_name(),(finish-start))

    # rest
    rospy.loginfo("%s : rest",rospy.get_name())
    restGoal = RestActionGoal()
    mp.rest_client.send_goal(restGoal)
    result = mp.rest_client.get_result()
    rospy.loginfo("%s : rest result : %s",rospy.get_name(), result)