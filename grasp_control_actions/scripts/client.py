# write a an action client for the rests
import rospy
from grasp_control_actions.msg import PlaceMsgAction, PlaceMsgActionGoal, PlaceMsgActionFeedback, PlaceMsgActionResult, PlaceMsgGoal
from grasp_control_actions.msg import PlaceVelAction, PlaceVelActionGoal, PlaceVelActionFeedback, PlaceVelActionResult, PlaceVelGoal
from grasp_control_actions.msg import RadialTrackingAction, RadialTrackingGoal, RadialTrackingActionGoal, RadialTrackingActionFeedback, RadialTrackingActionResult
from grasp_control_actions.msg import YoinkAction, YoinkActionGoal, YoinkActionFeedback, YoinkActionResult
from grasp_control_actions.msg import RestAction, RestActionGoal, RestActionFeedback, RestActionResult
from geometry_msgs.msg import PoseStamped, Twist
import actionlib
import threading
from ur_msgs.srv import SetIO, SetIORequest
import threading
import copy
from std_msgs.msg import Float32

YOINK_FEEDBACK_THRESHOLD = 0.05
VELOCITY_AVG_WINDOW_SIZE = 10
ALPHA = 0.5 # more means trust filtered data more
Z_DISPLACEMENT = 0.20 #m, should reduce to make it faster
MAX_VELOCITY_THRESHOLD = 0.07 #ms-1, should reduce to make it faster
HANDOVER_VELOCITY_THRESHOLD = 0.07 #ms-1 should increase to make it faster

class MpClass:
    def __init__(self):
        rospy.init_node("motion_planning_client")

        # other variables
        self.yoink_feedback = None
        self.yoink_feedback_threshold = YOINK_FEEDBACK_THRESHOLD
        self.fgp = None
        self.fgp_inital = None
        self.fgp_list = []
        self.fgp_linear_velocity = Twist()
        self.fgp_linear_velocity_filtered = Twist()
        self._velocity_filter_coeff = ALPHA
        self.array_access_lock = threading.Lock()
        self._max_vel_z = 0.0
        self.start_handover = 0.0 # this is just for debugging
        
        # # connect to action servers
        # rospy.loginfo("%s Started client, connecting to action servers", rospy.get_name())
        # self.place_client = actionlib.SimpleActionClient("place_server", PlaceMsgAction)
        # self.place_vel_client = actionlib.SimpleActionClient("place_vel", PlaceVelAction)
        # self.rest_client = actionlib.SimpleActionClient("rest", RestAction)
        # self.radial_tracking_client = actionlib.SimpleActionClient("radial_track", RadialTrackingAction)
        # self.yoink_client = actionlib.SimpleActionClient("yoink", YoinkAction)
        
        # rospy.loginfo("%s Waiting for clients", rospy.get_name())
        
        # client_connection1 = self.yoink_client.wait_for_server(timeout=rospy.Duration(secs=3))
        # client_connection2 = self.radial_tracking_client.wait_for_server(timeout=rospy.Duration(secs=3))
        # client_connection3 = self.rest_client.wait_for_server(timeout=rospy.Duration(secs=3))
        # client_connection4 = self.place_client.wait_for_server(timeout=rospy.Duration(secs=3))
        # client_connection5 = self.place_vel_client.wait_for_server(timeout=rospy.Duration(secs=3))
        
        # # connect to ur pin service to commands gripper pins
        # self.set_io_client = rospy.ServiceProxy("/ur_hardware_interface/set_io", SetIO)
        # self.set_io_client.wait_for_service(5.0)
        # self.yoink_feedback_sub = rospy.Subscriber("/yoink/linear_error",Float32,callback=self.yoink_feedback_cb)

        self.fgp_sub = rospy.Subscriber("/filtered_grasp_pose",PoseStamped,callback=self.fgp_cb)
        self.fgp_linear_velocity_publisher = rospy.Publisher("/filtered_grasp_pose_linear_velocity",Twist,queue_size=1)
        self.start_handover_publisher = rospy.Publisher("/start_handover",Float32,queue_size=1)
        rospy.loginfo("Waiting for one fgp message")
        rospy.wait_for_message("/filtered_grasp_pose",PoseStamped)
        rospy.loginfo("Message received")
        self.fgp_inital = copy.deepcopy(self.fgp)

        # if not (client_connection1 and client_connection2 and client_connection3 and client_connection4 and client_connection5):
        #     rospy.logwarn("%s Some clients are not connected!!",rospy.get_name())
        # else : 
        #     rospy.loginfo("%s : All servers connected",rospy.get_name())

        self.calculate_fgp_linear_velocity_timer = rospy.Timer(rospy.Duration(1/30),self.calculate_fgp_linear_velocity_cb)
        self.filter_fgp_vel_timer = rospy.Timer(rospy.Duration(1/30),self.filter_fgp_vel_cb)

    def calculate_fgp_linear_velocity_cb(self,event):
        with self.array_access_lock:
            if len(self.fgp_list)-1 > 0:
                if len(self.fgp_list) < VELOCITY_AVG_WINDOW_SIZE:
                    delta_t = self.fgp_list[-1].header.stamp.to_sec() - self.fgp_list[0].header.stamp.to_sec()
                    delta_x = self.fgp_list[-1].pose.position.x - self.fgp_list[0].pose.position.x
                    delta_y = self.fgp_list[-1].pose.position.y - self.fgp_list[0].pose.position.y
                    delta_z = self.fgp_list[-1].pose.position.z - self.fgp_list[0].pose.position.z

                    self.fgp_linear_velocity.linear.x = delta_x/delta_t
                    self.fgp_linear_velocity.linear.y = delta_y/delta_t
                    self.fgp_linear_velocity.linear.z = delta_z/delta_t
                else :
                    delta_t = self.fgp_list[-1].header.stamp.to_sec() - self.fgp_list[-VELOCITY_AVG_WINDOW_SIZE].header.stamp.to_sec()
                    delta_x = self.fgp_list[-1].pose.position.x - self.fgp_list[-VELOCITY_AVG_WINDOW_SIZE].pose.position.x
                    delta_y = self.fgp_list[-1].pose.position.y - self.fgp_list[-VELOCITY_AVG_WINDOW_SIZE].pose.position.y
                    delta_z = self.fgp_list[-1].pose.position.z - self.fgp_list[-VELOCITY_AVG_WINDOW_SIZE].pose.position.z

                    self.fgp_linear_velocity.linear.x = delta_x/delta_t
                    self.fgp_linear_velocity.linear.y = delta_y/delta_t
                    self.fgp_linear_velocity.linear.z = delta_z/delta_t
        if self.fgp_linear_velocity.linear.z > self._max_vel_z:
            self._max_vel_z = self.fgp_linear_velocity.linear.z

    def filter_fgp_vel_cb(self,event):
        self.fgp_linear_velocity_filtered.linear.x  = (1-self._velocity_filter_coeff)*self.fgp_linear_velocity.linear.x + self._velocity_filter_coeff*self.fgp_linear_velocity_filtered.linear.x
        self.fgp_linear_velocity_filtered.linear.y  = (1-self._velocity_filter_coeff)*self.fgp_linear_velocity.linear.y + self._velocity_filter_coeff*self.fgp_linear_velocity_filtered.linear.y
        self.fgp_linear_velocity_filtered.linear.z  = (1-self._velocity_filter_coeff)*self.fgp_linear_velocity.linear.z + self._velocity_filter_coeff*self.fgp_linear_velocity_filtered.linear.z
    
    def yoink_feedback_cb(self,msg):
        self.yoink_feedback = msg

    def fgp_cb(self,msg):
        self.fgp = msg
        with self.array_access_lock:
            self.fgp_list.append(self.fgp)
        self.fgp_linear_velocity_publisher.publish(self.fgp_linear_velocity_filtered)
        self.start_handover_publisher.publish(Float32(data=self.start_handover))

    def wait_for_handover(self):
        rate = rospy.Rate(50)
        start = rospy.get_time()

        while not rospy.is_shutdown():
            # if (self.fgp_linear_velocity_filtered.linear.z < HANDOVER_VELOCITY_THRESHOLD and self._max_vel_z > MAX_VELOCITY_THRESHOLD and 
            #    self.fgp.pose.position.z - self.fgp_inital.pose.position.z > Z_DISPLACEMENT):
            if (self.fgp.pose.position.z - self.fgp_inital.pose.position.z > Z_DISPLACEMENT and 
                # self.fgp_linear_velocity_filtered.linear.z < HANDOVER_VELOCITY_THRESHOLD and 
                self._max_vel_z > MAX_VELOCITY_THRESHOLD):
                self.start_handover = 0.5
                break
            else :
                self.start_handover = 0.0

            rate.sleep()

        rospy.loginfo(f"Done waiting for handover, waited {rospy.get_time()-start}")
                
    def wait_to_grasp(self):
        rate = rospy.Rate(30)
        rospy.loginfo(f"{rospy.get_name()} : waiting for grasp")
        rospy.wait_for_message("/yoink/linear_error",Float32,3.0)
        while self.yoink_feedback > self.yoink_feedback_threshold:
            rate.sleep()

    def gripper_on(self):
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT2,1) #positive to reinforce grip
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT3,0) #vacuum to open more
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT6,1) #vacuum to close
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT7,1) #vacuum for suction cup

    def gripper_neutral(self):
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT2,0) #positive to reinforce grip
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT3,0) #vacuum to open more
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT6,0) #vacuum to close
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT7,0) #vacuum for suction cup

    def gripper_off(self):
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT2,0)
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT3,1)
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT6,0)
        self.set_io_client(1,SetIORequest.PIN_CONF_OUT7,0)


if __name__ == "__main__":
    mp = MpClass()

    mp.wait_for_handover()
    
    rospy.spin()
    # # rest
    # rospy.loginfo("%s : rest",rospy.get_name())
    # restGoal = RestActionGoal()
    # mp.rest_client.send_goal(restGoal)
    # result = mp.rest_client.get_result()
    # rospy.loginfo("%s : rest result : %s",rospy.get_name(), result)

    # # rest
    # rospy.loginfo("%s : rest",rospy.get_name())
    # restGoal = RestActionGoal()
    # mp.rest_client.send_goal(restGoal)
    # result = mp.rest_client.get_result()
    # rospy.loginfo("%s : rest result : %s",rospy.get_name(), result)

    # start = rospy.get_time()
    # print("Gonna start radial tracking now")
    # rospy.sleep(2)
    # # radial track for a few seconds
    # rospy.loginfo("%s : radial tracking",rospy.get_name())
    # radial_trackGoal = RadialTrackingGoal()
    # radial_trackGoal.timeout.data = 0.0
    # mp.radial_tracking_client.send_goal(radial_trackGoal)
    # mp.radial_tracking_client.wait_for_result()
    # result = mp.radial_tracking_client.get_result()
    # rospy.loginfo("%s : radial tracking result : %s",rospy.get_name(), result)
    # rospy.sleep(2)
    # print("Radial tracking stopped")
    
    # # yoink
    # rospy.loginfo("%s : yoink",rospy.get_name())
    # yoinkGoal = YoinkActionGoal()
    # mp.yoink_client.send_goal(yoinkGoal)
    # mp.wait_to_grasp()
    # mp.gripper_on()
    # mp.yoink_client.wait_for_result()
    # result = mp.yoink_client.get_result()
    # rospy.loginfo("%s : yoink result : %s",rospy.get_name(), result)

    # # place
    # rospy.loginfo("%s : place",rospy.get_name())
    # placeGoal = PlaceMsgGoal()
    # mp.place_client.send_goal(placeGoal)
    # mp.place_client.wait_for_result()
    # result = mp.place_client.get_result()
    # rospy.loginfo("%s : place result : %s",rospy.get_name(), result)
    
    # # place vel
    # rospy.loginfo("%s : place",rospy.get_name())
    # placeGoal = PlaceVelGoal()
    # mp.place_vel_client.send_goal(placeGoal)
    # # mp.place_client.cancel_goal()
    # mp.place_vel_client.wait_for_result()
    # result = mp.place_vel_client.get_result()
    # rospy.loginfo("%s : place result : %s",rospy.get_name(), result)

    # finish = rospy.get_time()
    # # report
    # rospy.loginfo("%s : time taken to finish is %s",rospy.get_name(),(finish-start))

    # # rest
    # rospy.loginfo("%s : rest",rospy.get_name())
    # restGoal = RestActionGoal()
    # mp.rest_client.send_goal(restGoal)
    # result = mp.rest_client.get_result()
    # rospy.loginfo("%s : rest result : %s",rospy.get_name(), result)

    # # rest
    # rospy.loginfo("%s : rest",rospy.get_name())
    # restGoal = RestActionGoal()
    # mp.rest_client.send_goal(restGoal)
    # result = mp.rest_client.get_result()
    # rospy.loginfo("%s : rest result : %s",rospy.get_name(), result)