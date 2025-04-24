import rospy
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension
from std_srvs.srv import SetBool, SetBoolResponse
from gripper_driver.srv import SetGripper, SetGripperRequest, SetGripperResponse

gripper1_initial_state = [0,0,0,0,0,0,0,0,0,0,0,1,0]
gripper2_initial_state = [1,0,0,0,0,0,0,0,0,0,0,0,0]

class RosServiceWrapper:
    def __init__(self):
        rospy.init_node('ros_service_wrapper')
        self.service = rospy.Service('/gripper1', SetGripper, self.gripper1_callback)
        self.service = rospy.Service('/gripper2', SetGripper, self.gripper2_callback)

        self.gripper1_setpoint_pub = rospy.Publisher('/gripper1_setpoint', UInt8MultiArray, queue_size=1)
        self.gripper2_setpoint_pub = rospy.Publisher('/gripper2_setpoint', UInt8MultiArray, queue_size=1)

        self.gripper1_setpoint = UInt8MultiArray()
        self.gripper1_setpoint.layout.data_offset = 0

        gripper1_dim = MultiArrayDimension()
        gripper1_dim.label = "header_22_46"
        gripper1_dim.size = 13
        gripper1_dim.stride = 13
        self.gripper1_setpoint.layout.dim.append(gripper1_dim)
        self.gripper1_setpoint.data = gripper1_initial_state

        self.gripper2_setpoint = UInt8MultiArray()
        self.gripper2_setpoint.layout.data_offset = 0
        gripper2_dim = MultiArrayDimension()
        gripper2_dim.label = "header_23_47"
        gripper2_dim.size = 13
        gripper2_dim.stride = 13
        self.gripper2_setpoint.layout.dim.append(gripper2_dim)
        self.gripper2_setpoint.data = gripper2_initial_state

        self.setpoint_pub_timer = rospy.Timer(rospy.Duration(1/15), self.setpoint_pub_callback)

    def gripper1_callback(self, req:SetGripperRequest):
        self.gripper1_setpoint = req.setpoint
        return SetGripperResponse(result=True)
    
    def gripper2_callback(self, req):
        self.gripper2_setpoint = req.setpoint
        return SetGripperResponse(result=True)

    def setpoint_pub_callback(self, event):
        self.gripper1_setpoint_pub.publish(self.gripper1_setpoint)
        self.gripper2_setpoint_pub.publish(self.gripper2_setpoint)

if __name__ == "__main__":
    RosServiceWrapper()
    rospy.loginfo("ROS Service Wrapper for gripper started")
    rospy.spin()