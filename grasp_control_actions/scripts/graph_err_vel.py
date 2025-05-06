import rospy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import PoseStamped, Twist

# Initialize global variables to store data
data1 = []
data2 = []
data3 = []
data4 = []
data5 = []

data1_unit = None
data2_unit = None
data3_unit = None
data4_unit = None
data5_unit = None

def callback1(msg:PoseStamped):
    global data1_unit
    global data2_unit
    global data3_unit
    
    data1_unit = msg.pose.position.x
    data2_unit = msg.pose.position.y
    data3_unit = msg.pose.position.z

def callback2(msg:Twist):
    global data4_unit
    data4_unit = msg.linear.z

def callback3(msg:Float32):
    global data5_unit
    data5_unit = msg.data

def plot_data(i):
    plt.cla()
    plt.plot(data1, label='x')
    plt.plot(data2, label='y')
    plt.plot(data3, label='z')
    plt.plot(data4, label='vz')
    plt.plot(data5, label='start')
    plt.legend(loc='upper left')
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.title('err-vel plot')

def appender_cb(event):
    data1.append(data1_unit)
    data2.append(data2_unit)
    data3.append(data3_unit)
    data4.append(data4_unit)
    data5.append(data5_unit)

def listener():
    rospy.init_node('plotter_node', anonymous=True)
    rospy.Subscriber('/filtered_grasp_pose', PoseStamped, callback1)
    rospy.Subscriber('/filtered_grasp_pose_linear_velocity', Twist, callback2)
    rospy.Subscriber('/start_handover', Float32, callback3)
    print("Started subscriber")
    rospy.wait_for_message("/filtered_grasp_pose",PoseStamped)
    rospy.wait_for_message("/filtered_grasp_pose_linear_velocity",PoseStamped)
    rospy.wait_for_message("/start_handover",Float32)
    appender_timer = rospy.Timer(rospy.Duration(1/30),appender_cb)
    
    ani = FuncAnimation(plt.gcf(), plot_data, interval=50)
    plt.tight_layout()
    plt.show()

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass