import rospy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import PoseStamped

# Initialize global variables to store data
data1 = []
# data2 = []

def callback1(msg:PoseStamped):
    data1.append(msg.pose.position.z)
    print("data : ",msg.pose.position.z)

# def callback2(msg):
#     # Append data from topic2
#     data2.append(msg.data)

def plot_data(i):
    # Clear the current plot
    plt.cla()
    # Plot data from both topics
    plt.plot(data1, label='z')
    # plt.plot(data2, label='linear_velocity')
    plt.legend(loc='upper left')
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.title('err-vel plot')

def listener():
    # Initialize the ROS node
    rospy.init_node('plotter_node', anonymous=True)
    rospy.Subscriber('/filtered_grasp_pose', PoseStamped, callback1)
    print("Started subscriber")
    # rospy.Subscriber('/linear_velocity', Float32, callback2)
    # rospy.wait_for_message("/linear_error",Float32)
    
    rospy.wait_for_message("/filtered_grasp_pose",PoseStamped)
    ani = FuncAnimation(plt.gcf(), plot_data, interval=50)

    plt.tight_layout()
    plt.show()

    # Keep the script running
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass