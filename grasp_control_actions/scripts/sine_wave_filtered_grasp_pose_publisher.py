import rospy
import math
from geometry_msgs.msg import PoseStamped


rospy.init_node("sine_wave_setpoint_publisher")

filtered_grasp_pose = PoseStamped()
filtered_grasp_pose.header.frame_id = "world"

filtered_grasp_pose.pose.position.x = 0.8
filtered_grasp_pose.pose.position.y = 0.1241
filtered_grasp_pose.pose.position.z = 1.3621
filtered_grasp_pose.pose.orientation.x = 0.3298
filtered_grasp_pose.pose.orientation.y = 0.3192
filtered_grasp_pose.pose.orientation.z = 0.6116
filtered_grasp_pose.pose.orientation.w = 0.6443

filtered_grasp_pose_publisher = rospy.Publisher("/filtered_grasp_pose",PoseStamped,queue_size=10)

amplitude = 0.05 #m
timer = 0.0
rate = rospy.Rate(50)
timer_multiplier = 0.000001
alpha = 50 # range from 1 to 100

while not rospy.is_shutdown():
    timer+=rospy.get_time()*timer_multiplier*alpha
    
    x = 0.8 + (amplitude/2)*(math.sin(timer))
    y = 0.1241 + (amplitude/2)*(math.cos(timer))
    
    filtered_grasp_pose.pose.position.x = x
    # filtered_grasp_pose.pose.position.y = y
    
    filtered_grasp_pose_publisher.publish(filtered_grasp_pose)
    rate.sleep()