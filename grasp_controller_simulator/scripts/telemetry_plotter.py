#!/usr/bin/env python3
# plots the current pose as a trajectory and the linear and rotational component of current velocity
import rospy
import threading
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseStamped, Twist
import tf.transformations as tf_trans
import numpy as np
import os

class PosePlotter:
    def __init__(self):
        rospy.init_node("pose_plotter", anonymous=True)

        self.pose_data = []
        self.pose_orientations = []
        self.current_lin_vel = []
        self.current_ang_vel = []

        self.recording = False
        self.lock = threading.Lock()

        rospy.Subscriber("/pose_controller/current_pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/pose_controller/current_velocity", Twist, self.current_velocity_callback)

        print("Press Enter to start/stop recording data...")
        self.listen_for_enter()

    def pose_callback(self, msg):
        if self.recording:
            with self.lock:
                self.pose_data.append((
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z
                ))
                self.pose_orientations.append((
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w
                ))

    def current_velocity_callback(self, msg):
        if self.recording:
            with self.lock:
                self.current_lin_vel.append((
                    msg.linear.x,
                    msg.linear.y,
                    msg.linear.z
                ))
                self.current_ang_vel.append((
                    msg.angular.x,
                    msg.angular.y,
                    msg.angular.z
                ))

    def listen_for_enter(self):
        def wait_for_input():
            while not rospy.is_shutdown():
                input()
                self.toggle_recording()
        t = threading.Thread(target=wait_for_input)
        t.daemon = True
        t.start()
        rospy.spin()

    def toggle_recording(self):
        self.recording = not self.recording
        if self.recording:
            print("Recording started...")
            self.clear_data()
        else:
            print("Recording stopped. Saving plots...")
            self.plot_and_save()

    def clear_data(self):
        with self.lock:
            self.pose_data.clear()
            self.pose_orientations.clear()
            self.current_lin_vel.clear()
            self.current_ang_vel.clear()

    def plot_and_save(self):
        with self.lock:
            if len(self.pose_data) == 0:
                print("No data recorded.")
                return

            timestamp = rospy.get_time()
            output_dir = os.path.join(os.getcwd(), f"plots_{int(timestamp)}")
            os.makedirs(output_dir, exist_ok=True)

            # --- 3D Trajectory Plot with Start/End Axes ---
            fig1 = plt.figure()
            ax = fig1.add_subplot(111, projection='3d')
            x, y, z = zip(*self.pose_data)
            ax.plot(x, y, z, label='Trajectory', color='blue')

            # Plot start pose axes
            self.plot_pose_axes(ax, self.pose_data[0], self.pose_orientations[0], label="Start Pose")
            # Plot end pose axes
            self.plot_pose_axes(ax, self.pose_data[-1], self.pose_orientations[-1], label="End Pose")

            ax.set_title('3D Trajectory with Start/End Axes')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.legend()
            ax.grid(True)
            fig1.tight_layout()
            fig1.savefig(os.path.join(output_dir, "trajectory_3d.png"))
            plt.close(fig1)

            # --- Linear Velocity Plot ---
            fig2, ax2 = plt.subplots()
            t = range(len(self.current_lin_vel))
            cx, cy, cz = zip(*self.current_lin_vel)

            ax2.plot(t, cx, label='Current Linear X', color='blue')
            ax2.plot(t, cy, label='Current Linear Y', color='green')
            ax2.plot(t, cz, label='Current Linear Z', color='red')

            ax2.set_title('Linear Velocity (X, Y, Z)')
            ax2.set_xlabel('Time Step')
            ax2.set_ylabel('Velocity (m/s)')
            ax2.grid(True)
            ax2.legend()
            fig2.tight_layout()
            fig2.savefig(os.path.join(output_dir, "linear_velocity.png"))
            plt.close(fig2)

            # --- Angular Velocity Plot (RPY) ---
            fig3, ax3 = plt.subplots()
            r, p, y_ = zip(*self.current_ang_vel)

            ax3.plot(t, r, label='Current Roll', color='blue')
            ax3.plot(t, p, label='Current Pitch', color='green')
            ax3.plot(t, y_, label='Current Yaw', color='red')

            ax3.set_title('Angular Velocity (RPY)')
            ax3.set_xlabel('Time Step')
            ax3.set_ylabel('Angular Velocity (rad/s)')
            ax3.grid(True)
            ax3.legend()
            fig3.tight_layout()
            fig3.savefig(os.path.join(output_dir, "angular_velocity.png"))
            plt.close(fig3)

            print(f"Plots saved in: {output_dir}")

    def plot_pose_axes(self, ax, position, orientation, label=""):
        origin = np.array(position)
        quat = orientation
        rot_matrix = tf_trans.quaternion_matrix(quat)[:3, :3]
        length = 0.02  # arrow length

        # Draw axis arrows
        for i, (axis, color) in enumerate(zip(rot_matrix.T, ['red', 'green', 'blue'])):
            ax.quiver(*origin, *axis*length, color=color, linewidth=2)

        # Add scatter for the pose position
        ax.scatter(*origin, color='black', label=label, s=30)

if __name__ == "__main__":
    try:
        PosePlotter()
    except rospy.ROSInterruptException:
        pass
