# write a an action client for the rests
import rospy
from grasp_control_actions.msg import PlaceMsgAction, PlaceMsgActionGoal, PlaceMsgActionFeedback, PlaceMsgActionResult, PlaceMsgGoal
from grasp_control_actions.msg import PlaceVelAction, PlaceVelActionGoal, PlaceVelActionFeedback, PlaceVelActionResult, PlaceVelGoal
from grasp_control_actions.msg import RadialTrackingAction, RadialTrackingGoal, RadialTrackingActionGoal, RadialTrackingActionFeedback, RadialTrackingActionResult
from grasp_control_actions.msg import YoinkAction, YoinkActionGoal, YoinkActionFeedback, YoinkActionResult
from grasp_control_actions.msg import RestAction, RestActionGoal, RestActionFeedback, RestActionResult
import actionlib
import threading
from ur_msgs.srv import SetIO, SetIORequest
from std_msgs.msg import Float32

class MpClass:
    def __init__(self):
        rospy.init_node("motion_planning_client")

        rospy.loginfo("%s Started client, connecting to action servers", rospy.get_name())
        
        # connect to action servers
        self.place_client = actionlib.SimpleActionClient("place_server", PlaceMsgAction)
        self.place_br_client = actionlib.SimpleActionClient("place_blended_radius_server", PlaceMsgAction)
        self.place_vel_client = actionlib.SimpleActionClient("place_vel", PlaceVelAction)
        self.place_traj_vel_client = actionlib.SimpleActionClient("place_traj_vel", PlaceVelAction)
        self.rest_client = actionlib.SimpleActionClient("rest", RestAction)
        self.radial_tracking_client = actionlib.SimpleActionClient("radial_track", RadialTrackingAction)
        self.yoink_client = actionlib.SimpleActionClient("yoink", YoinkAction)
        
        rospy.loginfo("%s Waiting for clients", rospy.get_name())
        
        client_connection1 = self.yoink_client.wait_for_server(timeout=rospy.Duration(secs=3))
        client_connection2 = self.radial_tracking_client.wait_for_server(timeout=rospy.Duration(secs=3))
        client_connection3 = self.rest_client.wait_for_server(timeout=rospy.Duration(secs=3))
        client_connection4 = self.place_client.wait_for_server(timeout=rospy.Duration(secs=3))
        client_connection5 = self.place_br_client.wait_for_server(timeout=rospy.Duration(secs=3))
        client_connection6 = self.place_vel_client.wait_for_server(timeout=rospy.Duration(secs=3))
        
        # connect to ur pin service to commands gripper pins
        # self.set_io_client = rospy.ServiceProxy("/ur_hardware_interface/set_io", SetIO)
        # self.set_io_client.wait_for_service(5.0)
        self.yoink_feedback_sub = rospy.Subscriber("/yoink/linear_error",Float32,callback=self.yoink_feedback_cb)
        self.place_br_feedback_sub = rospy.Subscriber("/place_br/linear_error",Float32,callback=self.place_br_feedback_cb)

        # action feedback variables
        self.yoink_feedback = None
        self.yoink_feedback_threshold = 0.1
        self.place_br_feedback = None
        self.place_br_feedback_threshold = 0.1

        if not (client_connection1 and client_connection2 and client_connection3 and client_connection4 and client_connection5 and client_connection6):
            rospy.logwarn("%s Some clients are not connected!!",rospy.get_name())
        else : 
            rospy.loginfo("%s : All servers connected",rospy.get_name())

    def yoink_feedback_cb(self,msg):
        self.yoink_feedback = msg

    def place_br_feedback_cb(self,msg):
        self.place_br_feedback = msg
    
    def wait_to_grasp(self):
        rate = rospy.Rate(30)
        rospy.loginfo(f"{rospy.get_name()} : waiting for grasp")
        rospy.wait_for_message("/yoink/linear_error",Float32,3.0)
        start = rospy.get_time()
        rospy.sleep(0.2)
        while self.yoink_feedback.data > self.yoink_feedback_threshold:
            rate.sleep()
        rospy.sleep(0.5)
        print("Waited for : ",rospy.get_time() - start)

    def wait_to_release(self):
        rate = rospy.Rate(30)
        rospy.loginfo(f"{rospy.get_name()} : waiting for release")
        rospy.wait_for_message("/place_br/linear_error",Float32,3.0)
        start = rospy.get_time()
        rospy.sleep(0.2)
        while self.place_br_feedback.data > self.place_br_feedback_threshold:
            rate.sleep()
        rospy.sleep(0.5)
        print("Waited for : ",rospy.get_time() - start)

    def gripper_on(self):
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT2,1)
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT3,0)
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT6,1)
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT7,1)

    def gripper_off(self):
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT2,0)
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT3,1)
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT6,0)
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT7,0)


if __name__ == "__main__":
    mp = MpClass()
    
    start = rospy.get_time()

    # rest
    rospy.loginfo("%s : rest",rospy.get_name())
    restGoal = RestActionGoal()
    mp.rest_client.send_goal(restGoal)
    result = mp.rest_client.get_result()
    mp.rest_client.wait_for_result()
    rospy.loginfo("%s : rest result : %s",rospy.get_name(), result)

    # # radial track for a few seconds
    # rospy.loginfo("%s : radial tracking",rospy.get_name())
    # radial_trackGoal = RadialTrackingGoal()
    # radial_trackGoal.timeout.data = 0.99
    # mp.radial_tracking_client.send_goal(radial_trackGoal)
    # mp.radial_tracking_client.wait_for_result()
    # result = mp.radial_tracking_client.get_result()
    # rospy.loginfo("%s : radial tracking result : %s",rospy.get_name(), result)

    # # yoink
    # rospy.loginfo("%s : yoink",rospy.get_name())
    # yoinkGoal = YoinkActionGoal()
    # mp.yoink_client.send_goal(yoinkGoal)
    # mp.wait_to_grasp()
    # # mp.gripper_on()
    # mp.yoink_client.wait_for_result()
    # result = mp.yoink_client.get_result()
    # rospy.loginfo("%s : yoink result : %s",rospy.get_name(), result)

    # # place
    # rospy.loginfo("%s : place",rospy.get_name())
    # placeGoal = PlaceMsgGoal()
    # mp.place_br_client.send_goal(placeGoal)
    # mp.place_br_client.wait_for_result()
    # result = mp.place_br_client.get_result()
    # rospy.loginfo("%s : place result : %s",rospy.get_name(), result)
    
    # # place vel
    # rospy.loginfo("%s : place",rospy.get_name())
    # placeGoal = PlaceVelGoal()
    # mp.place_vel_client.send_goal(placeGoal)
    # # mp.place_client.cancel_goal()
    # mp.place_vel_client.wait_for_result()
    # result = mp.place_vel_client.get_result()
    # rospy.loginfo("%s : place result : %s",rospy.get_name(), result)

    # # place traj vel 
    # rospy.loginfo("%s : place",rospy.get_name())
    # placeGoal = PlaceVelGoal()
    # mp.place_traj_vel_client.send_goal(placeGoal)
    # # mp.place_client.cancel_goal()
    # mp.place_traj_vel_client.wait_for_result()
    # result = mp.place_traj_vel_client.get_result()
    # rospy.loginfo("%s : place result : %s",rospy.get_name(), result)

    finish = rospy.get_time()
    # report
    rospy.loginfo("%s : time taken to finish is %s",rospy.get_name(),(finish-start))

    # rest
    rospy.loginfo("%s : rest",rospy.get_name())
    restGoal = RestActionGoal()
    mp.rest_client.send_goal(restGoal)
    result = mp.rest_client.get_result()
    mp.rest_client.wait_for_result()
    rospy.loginfo("%s : rest result : %s",rospy.get_name(), result)