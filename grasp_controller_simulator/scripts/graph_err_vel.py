import rospy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Initialize global variables to store data
data1 = []
data2 = []

def callback1(msg):
    # Append data from topic1
    data1.append(msg.data)

def callback2(msg):
    # Append data from topic2
    data2.append(msg.data)

def plot_data(i):
    # Clear the current plot
    plt.cla()
    # Plot data from both topics
    plt.plot(data1, label='linear_error')
    plt.plot(data2, label='linear_velocity')
    plt.legend(loc='upper left')
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.title('err-vel plot')

def listener():
    # Initialize the ROS node
    rospy.init_node('plotter_node', anonymous=True)
    
    # Subscribe to the two topics
    rospy.Subscriber('/linear_error', Float32, callback1)
    rospy.Subscriber('/linear_velocity', Float32, callback2)
    
    # Set up the plot
    rospy.wait_for_message("/linear_error",Float32)
    rospy.wait_for_message("/linear_velocity",Float32)
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