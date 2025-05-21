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
from gripper_driver.srv import SetGripper, SetGripperRequest
import numpy as np
import tf2_ros, tf2_geometry_msgs

'''
rest state
  base: 3.180511713027954
  shoulder: -0.765713707809784
  elbow: -2.063561201095581
  wrist1: -3.4858814678587855
  wrist2: -1.6338766256915491
  wrist3: -0.026324097310201466
'''

YOINK_FEEDBACK_THRESHOLD = 0.1
PLACE_FEEDBACK_THRESHOLD = 0.05 # made it super high to test
VELOCITY_AVG_WINDOW_SIZE = 10
ALPHA = 0.5 # more means trust filtered data more
Z_DISPLACEMENT = 0.2 #m, should reduce to make it faster
MAX_VELOCITY_THRESHOLD = 0.0 #ms-1, should reduce to make it faster
HANDOVER_VELOCITY_THRESHOLD = 0.5 #ms-1 should increase to make it faster
MIN_HAND_Z = 0.05
HAND_PROXIMITY_THRESHOLD = 0.15 # m, should be higher to start sooner
PROXIMITY_BASED_WAIT_LOWER_BOUND = 0.2 #m, this is used to decide if we choose to perform handvelbasedwait

class MpClass:
    def __init__(self):
        rospy.init_node("motion_planning_client")
        rospy.loginfo("%s Started client, connecting to action servers", rospy.get_name())

        # other variables
        self.yoink_feedback = None
        self.yoink_feedback_threshold = YOINK_FEEDBACK_THRESHOLD
        self.place_br_feedback = None
        self.place_br_feedback_threshold = PLACE_FEEDBACK_THRESHOLD
        self.fgp = None
        self.fgp_inital = None
        self.fgp_list = []
        self.fgp_linear_velocity = Twist()
        self.fgp_linear_velocity_filtered = Twist()
        self.hand = None
        self.hand_inital = None
        self.hand_list = []
        self.hand_linear_velocity = Twist()
        self.hand_linear_velocity_filtered = Twist()
        self._velocity_filter_coeff = ALPHA
        self.array_access_lock = threading.Lock()
        self._max_vel_z = 0.0
        self.start_handover = 0.0 # this is just for debugging
        
        # connect to action servers
        rospy.loginfo("%s Started client, connecting to action servers", rospy.get_name())
        self.place_client = actionlib.SimpleActionClient("place_server", PlaceMsgAction)
        self.place_vel_client = actionlib.SimpleActionClient("place_vel", PlaceVelAction)
        self.place_br_client = actionlib.SimpleActionClient("place_blended_radius_server", PlaceMsgAction)
        self.rest_client = actionlib.SimpleActionClient("rest", RestAction)
        self.radial_tracking_client = actionlib.SimpleActionClient("radial_track", RadialTrackingAction)
        self.yoink_client = actionlib.SimpleActionClient("yoink", YoinkAction)
        self.yoink_v2_client = actionlib.SimpleActionClient("yoink_v2", YoinkAction)
        
        rospy.loginfo("%s Waiting for clients", rospy.get_name())
        
        client_connection1 = self.yoink_client.wait_for_server(timeout=rospy.Duration(secs=3))
        client_connection2 = self.yoink_v2_client.wait_for_server(timeout=rospy.Duration(secs=3))
        client_connection3 = self.radial_tracking_client.wait_for_server(timeout=rospy.Duration(secs=3))
        client_connection4 = self.rest_client.wait_for_server(timeout=rospy.Duration(secs=3))
        client_connection5 = self.place_client.wait_for_server(timeout=rospy.Duration(secs=3))
        client_connection6 = self.place_vel_client.wait_for_server(timeout=rospy.Duration(secs=3))
        client_connection7 = self.place_br_client.wait_for_server(timeout=rospy.Duration(secs=3))
        
        # connect to ur pin service to commands gripper pins
        self.set_io_client = rospy.ServiceProxy("/ur_hardware_interface/set_io", SetIO)
        self.set_io_client.wait_for_service(5.0)
        self.set_gripper_client = rospy.ServiceProxy("/gripper1",SetGripper)
        self.yoink_feedback_sub = rospy.Subscriber("/yoink/linear_error",Float32,callback=self.yoink_feedback_cb)
        self.yoink_feedback_sub = rospy.Subscriber("/place_vel/linear_error",Float32,callback=self.place_br_feedback_cb)
        self.fgp_sub = rospy.Subscriber("/filtered_grasp_pose",PoseStamped,callback=self.fgp_cb)
        self.hand_pose_sub = rospy.Subscriber("/hand_filtered_pose",PoseStamped,callback=self.hand_pose_cb)
        self.start_handover_publisher = rospy.Publisher("/start_handover",Float32,queue_size=1)
        self.fgp_linear_velocity_publisher = rospy.Publisher("/filtered_grasp_pose_linear_velocity",Twist,queue_size=1)
        self.calculate_fgp_linear_velocity_timer = rospy.Timer(rospy.Duration(1/30),self.calculate_fgp_linear_velocity_cb)
        self.filter_fgp_vel_timer = rospy.Timer(rospy.Duration(1/30),self.filter_fgp_vel_cb)
        self.hand_linear_velocity_publisher = rospy.Publisher("/hand_pose_linear_velocity",Twist,queue_size=1)
        self.calculate_hand_linear_velocity_timer = rospy.Timer(rospy.Duration(1/30),self.calculate_hand_linear_velocity_cb)
        self.filter_hand_vel_timer = rospy.Timer(rospy.Duration(1/30),self.filter_hand_vel_cb)

        # tf shit
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("Waiting for one fgp message")
        rospy.wait_for_message("/filtered_grasp_pose",PoseStamped)
        rospy.loginfo("Message received")
        rospy.loginfo("Waiting for one hand message")
        rospy.wait_for_message("/hand_pose",PoseStamped)
        rospy.loginfo("Message received")
        self.fgp_inital = copy.deepcopy(self.fgp)
        self.hand_inital = copy.deepcopy(self.hand)

        if not (client_connection1 and client_connection2 and client_connection3 and client_connection4 and client_connection5 and client_connection6 and client_connection7):
            rospy.logwarn("%s Some clients are not connected!!",rospy.get_name())
        else : 
            rospy.loginfo("%s : All servers connected",rospy.get_name())

    def transform_pose(self, point, target_frame):
        try:
            return self.tf_buffer.transform(point, target_frame, rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transform failed: {e}")
            return None

    def to_tf_msg(self,input:PoseStamped):
        result = tf2_geometry_msgs.PoseStamped()
        result.header.stamp = input.header.stamp
        result.header.frame_id = input.header.frame_id
        result.pose.position.x = input.pose.position.x
        result.pose.position.y = input.pose.position.y
        result.pose.position.z = input.pose.position.z
        result.pose.orientation.x = input.pose.orientation.x
        result.pose.orientation.y = input.pose.orientation.y
        result.pose.orientation.z = input.pose.orientation.z
        result.pose.orientation.w = input.pose.orientation.w
        return result

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
    
    def calculate_hand_linear_velocity_cb(self,event):
        with self.array_access_lock:
            if len(self.hand_list)-1 > 0:
                if len(self.hand_list) < VELOCITY_AVG_WINDOW_SIZE:
                    delta_t = self.hand_list[-1].header.stamp.to_sec() - self.hand_list[0].header.stamp.to_sec()
                    delta_x = self.hand_list[-1].pose.position.x - self.hand_list[0].pose.position.x
                    delta_y = self.hand_list[-1].pose.position.y - self.hand_list[0].pose.position.y
                    delta_z = self.hand_list[-1].pose.position.z - self.hand_list[0].pose.position.z

                    self.hand_linear_velocity.linear.x = delta_x/delta_t
                    self.hand_linear_velocity.linear.y = delta_y/delta_t
                    self.hand_linear_velocity.linear.z = delta_z/delta_t
                else :
                    delta_t = self.hand_list[-1].header.stamp.to_sec() - self.hand_list[-VELOCITY_AVG_WINDOW_SIZE].header.stamp.to_sec()
                    delta_x = self.hand_list[-1].pose.position.x - self.hand_list[-VELOCITY_AVG_WINDOW_SIZE].pose.position.x
                    delta_y = self.hand_list[-1].pose.position.y - self.hand_list[-VELOCITY_AVG_WINDOW_SIZE].pose.position.y
                    delta_z = self.hand_list[-1].pose.position.z - self.hand_list[-VELOCITY_AVG_WINDOW_SIZE].pose.position.z

                    self.hand_linear_velocity.linear.x = delta_x/delta_t
                    self.hand_linear_velocity.linear.y = delta_y/delta_t
                    self.hand_linear_velocity.linear.z = delta_z/delta_t
        if self.hand_linear_velocity.linear.z > self._max_vel_z:
            self._max_vel_z = self.hand_linear_velocity.linear.z

    def filter_hand_vel_cb(self,event):
        self.hand_linear_velocity_filtered.linear.x  = (1-self._velocity_filter_coeff)*self.hand_linear_velocity.linear.x + self._velocity_filter_coeff*self.hand_linear_velocity_filtered.linear.x
        self.hand_linear_velocity_filtered.linear.y  = (1-self._velocity_filter_coeff)*self.hand_linear_velocity.linear.y + self._velocity_filter_coeff*self.hand_linear_velocity_filtered.linear.y
        self.hand_linear_velocity_filtered.linear.z  = (1-self._velocity_filter_coeff)*self.hand_linear_velocity.linear.z + self._velocity_filter_coeff*self.hand_linear_velocity_filtered.linear.z
    

    def yoink_feedback_cb(self,msg):
        self.yoink_feedback = msg

    def place_br_feedback_cb(self,msg):
        self.place_br_feedback = msg

    def hand_pose_cb(self,msg):
        self.hand = self.transform_pose(self.to_tf_msg(msg),"base_link")

    def fgp_cb(self,msg):
        self.fgp = msg
        with self.array_access_lock:
            self.fgp_list.append(self.fgp)
        self.fgp_linear_velocity_publisher.publish(self.fgp_linear_velocity_filtered)
        self.start_handover_publisher.publish(Float32(data=self.start_handover))

    def wait_for_handover_hand_proximity_based(self):
        rate = rospy.Rate(30)
        rospy.loginfo(f"Waiting for hand proximity based handover ")
        start = rospy.get_time()
        while not rospy.is_shutdown():
            if self.hand is not None and self.fgp is not None :
                hand_position = np.array([self.hand.pose.position.x,self.hand.pose.position.y,self.hand.pose.position.z],dtype=float)
                object_position = np.array([self.fgp.pose.position.x,self.fgp.pose.position.y,self.fgp.pose.position.z],dtype=float)
                delta_d = np.linalg.norm(hand_position-object_position)
                if delta_d < HAND_PROXIMITY_THRESHOLD:
                    break
            rate.sleep()
        return rospy.get_time() - start
    

    def wait_for_handover_hand_vel_based(self):
        rate = rospy.Rate(50)
        start = rospy.get_time()
        rospy.loginfo(f"Waiting for hand vel based handover ")
        while not rospy.is_shutdown():
            if (self.hand.pose.position.z > MIN_HAND_Z and
                self.hand.pose.position.z - self.hand_inital.pose.position.z > Z_DISPLACEMENT and 
                self.hand_linear_velocity_filtered.linear.z < HANDOVER_VELOCITY_THRESHOLD and 
                self._max_vel_z > MAX_VELOCITY_THRESHOLD):
                self.start_handover = 0.5
                break
            else :
                self.start_handover = 0.0
            rate.sleep()

        rospy.loginfo(f"Done waiting for handover, waited {rospy.get_time()-start}")

    def wait_for_handover(self):
        rate = rospy.Rate(50)
        start = rospy.get_time()
        rospy.loginfo(f"Waiting for object vel based handover ")
        while not rospy.is_shutdown():
            # if self.hand is not None and self.fgp is not None and self.fgp_inital is not None and self.fgp_linear_velocity_filtered is not None :
            if (self.fgp.pose.position.z - self.fgp_inital.pose.position.z > Z_DISPLACEMENT and 
                self.fgp_linear_velocity_filtered.linear.z < HANDOVER_VELOCITY_THRESHOLD and 
                self._max_vel_z > MAX_VELOCITY_THRESHOLD):
                self.start_handover = 0.5
                break
            else :
                self.start_handover = 0.0

        rate.sleep()

        rospy.loginfo(f"Done waiting for handover, waited {rospy.get_time()-start}")

    def wait_for_handover_v3(self):
        rate = rospy.Rate(50)
        start = rospy.get_time()
        rospy.loginfo(f"Waiting for object vel based handover ")
        while not rospy.is_shutdown():
            # if self.hand is not None and self.fgp is not None and self.fgp_inital is not None and self.fgp_linear_velocity_filtered is not None :
            if (self.fgp.pose.position.z > Z_DISPLACEMENT and 
                self.fgp_linear_velocity_filtered.linear.z < HANDOVER_VELOCITY_THRESHOLD and 
                self._max_vel_z > MAX_VELOCITY_THRESHOLD):
                self.start_handover = 0.5
                break
            else :
                self.start_handover = 0.0

        rate.sleep()

        rospy.loginfo(f"Done waiting for handover, waited {rospy.get_time()-start}")

    def wait_for_handover_v2(self):
        rospy.loginfo(f"Waiting for wait for handover v2 ")
        start = rospy.get_time()
        delta_t = mp.wait_for_handover_hand_proximity_based()
        if delta_t < PROXIMITY_BASED_WAIT_LOWER_BOUND:
            mp.wait_for_handover_hand_vel_based()
        rospy.loginfo(f"Done waiting for handover, waited {rospy.get_time()-start}")
        
                
    def wait_to_grasp(self):
        rate = rospy.Rate(30)
        rospy.loginfo(f"{rospy.get_name()} : waiting for grasp")
        rospy.wait_for_message("/yoink/linear_error",Float32,3.0)
        rospy.sleep(0.1)
        start = rospy.get_time()
        rospy.sleep(0.3)
        while self.yoink_feedback.data > self.yoink_feedback_threshold:
            rate.sleep()
        rospy.sleep(0.5)
        print("Waited for : ",rospy.get_time() - start)
    
    def wait_to_release(self):
        rate = rospy.Rate(30)
        rospy.loginfo(f"{rospy.get_name()} : waiting for release")
        rospy.wait_for_message("/place_vel/linear_error",Float32)
        start = rospy.get_time()
        rospy.sleep(0.1)
        while self.place_br_feedback.data > self.place_br_feedback_threshold:
            rate.sleep()
        print("Waited for : ",rospy.get_time() - start)

    def gripper_on(self):
        # self.set_io_client(1,SetIORequest.PIN_CONF_OUT7,1) #positive to reinforce grip
        # self.set_io_client(1,SetIORequest.PIN_CONF_OUT6,0) #vacuum to open more
        # self.set_io_client(1,SetIORequest.PIN_CONF_OUT4,1) #vacuum to close
        # self.set_io_client(1,SetIORequest.PIN_CONF_OUT5,1) #vacuum for suction cup
        setpoint = SetGripperRequest()
        setpoint.setpoint.data = [0,1,1,0,1,0,0,0,0,0,0,0,0]
        self.set_gripper_client.call(setpoint)

    def gripper_neutral(self):
        # self.set_io_client(1,SetIORequest.PIN_CONF_OUT7,0) #positive to reinforce grip
        # self.set_io_client(1,SetIORequest.PIN_CONF_OUT6,0) #vacuum to open more
        # self.set_io_client(1,SetIORequest.PIN_CONF_OUT4,0) #vacuum to close
        # self.set_io_client(1,SetIORequest.PIN_CONF_OUT5,0) #vacuum for suction cup
        setpoint = SetGripperRequest()
        setpoint.setpoint.data = [0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.set_gripper_client.call(setpoint)

    def gripper_off(self):
        # self.set_io_client(1,SetIORequest.PIN_CONF_OUT7,0)
        # self.set_io_client(1,SetIORequest.PIN_CONF_OUT6,1)
        # self.set_io_client(1,SetIORequest.PIN_CONF_OUT4,0)
        # self.set_io_client(1,SetIORequest.PIN_CONF_OUT5,0)
        setpoint = SetGripperRequest()
        setpoint.setpoint.data = [1,0,0,0,0,0,0,0,0,0,0,0,0]
        self.set_gripper_client.call(setpoint)

def rest(mp):
    # rest
    rospy.loginfo("%s : rest",rospy.get_name())
    restGoal = RestActionGoal()
    mp.rest_client.send_goal(restGoal)
    mp.rest_client.wait_for_result()
    result = mp.rest_client.get_result()
    rospy.loginfo("%s : rest result : %s",rospy.get_name(), result)

def start_radial_track(mp):
    # radial track for a few seconds
    print("Gonna start radial tracking now")
    rospy.loginfo("%s : radial tracking",rospy.get_name())
    radial_trackGoal = RadialTrackingGoal()
    radial_trackGoal.timeout.data = 0.0
    mp.radial_tracking_client.send_goal(radial_trackGoal)
    mp.radial_tracking_client.wait_for_result() # uncomment this if you wanna block execution

def stop_radial_track(mp):
    print("Stopping radial tracking starting handover")
    mp.radial_tracking_client.cancel_goal()
    result = mp.radial_tracking_client.get_result()
    rospy.loginfo("%s : radial tracking result : %s",rospy.get_name(), result)
    print("Radial tracking stopped")

def yoink(mp):
    # yoink
    rospy.loginfo("%s : yoink",rospy.get_name())
    yoinkGoal = YoinkActionGoal()
    mp.yoink_client.send_goal(yoinkGoal)
    mp.wait_to_grasp()
    mp.gripper_on()
    mp.yoink_client.wait_for_result()
    result = mp.yoink_client.get_result()
    rospy.loginfo("%s : yoink result : %s",rospy.get_name(), result)

def yoink_v2(mp):
    # yoink
    rospy.loginfo("%s : yoink",rospy.get_name())
    yoinkGoal = YoinkActionGoal()
    mp.yoink_v2_client.send_goal(yoinkGoal)
    mp.wait_to_grasp()
    mp.gripper_on()
    mp.yoink_v2_client.wait_for_result()
    result = mp.yoink_v2_client.get_result()
    rospy.loginfo("%s : yoink result : %s",rospy.get_name(), result)

def place(mp):
    # place
    rospy.loginfo("%s : place",rospy.get_name())
    placeGoal = PlaceMsgGoal()
    mp.place_br_client.send_goal(placeGoal)
    print("waiting to release")
    mp.wait_to_release()
    print("done waiting to release")
    mp.gripper_off()
    finish_timestamp = rospy.get_time()
    mp.place_br_client.wait_for_result()
    result = mp.place_br_client.get_result()
    return finish_timestamp

def place_vel(mp):
    # place vel
    rospy.loginfo("%s : place",rospy.get_name())
    placeGoal = PlaceVelGoal()
    mp.place_vel_client.send_goal(placeGoal)
    print("waiting to release")
    mp.wait_to_release()
    print("done waiting to release")
    mp.gripper_off()
    finish_timestamp = rospy.get_time()
    mp.place_vel_client.wait_for_result()
    result = mp.place_vel_client.get_result()
    rospy.loginfo("%s : place result : %s",rospy.get_name(), result)
    return finish_timestamp

if __name__ == "__main__":
    mp = MpClass()
    rospy.sleep(0.2)

    #### mp functionality
    # mp.gripper_neutral()
    # mp.wait_for_handover()
    # mp.wait_for_handover_hand_proximity_based()
    # mp.wait_for_handover_hand_vel_based()
    # mp.wait_for_handover_v2()
    # mp.wait_to_grasp()
    # mp.gripper_off()
    # mp.gripper_on()
    
    # start_radial_track(mp)
    # rospy.sleep(10)
    # stop_radial_track(mp)
    # mp.gripper_neutral()
    
    rest(mp)
    mp.gripper_off()
    mp.wait_for_handover_v3()
    start = rospy.get_time()
    yoink(mp)
    rospy.sleep(0.5) # if its greater than 0.5, the start state for traj planning varies and traj fails
    finish  = place_vel(mp)
    rospy.loginfo("%s : time taken to finish is %s",rospy.get_name(),(finish-start))
    rest(mp)
    mp.gripper_neutral()

    # mp.gripper_off()
    # mp.gripper_on()
    # mp.gripper_neutral()