# HRH  2025

The following repo contains the solution for CORSMAL HRH 2025, stilll a WIP.</br>

Non dependency repositories include :
1. aruco_proxy_pose_estimation
2. calibration_targets
3. corsmal_setup_bringup
4. docs
5. grasp_control_actions
6. grasp_controller_simulation
7. gripper_driver
8. robot_setup_sim_bringup
9. robot_setup_sim_description

## Package descriptions
### 1. aruco_proxy_pose_estimation
**Description**: Package to simulate the 6Dof perception using aruco tags on boxes with hardcoded dimensions. Currently interfaced only with the realsense type cameras.

**Important Scripts**: 
1. `./scripts/create_marker.py` : creates a series of markers on an A4 sheet.
2. `./scripts/estimate_cube_pose_3d_bb.py` : Subscribes to the realsense topics, detects aruco pose, determines object pose and 3d BBox. Standalone script, does not depends on any other parameters, important interface parameters include the Image and CameraInfo topics, marker_size and tf_frame_id.

**Issues**:
1. The pose has high variance, needs to be filtered using EKF filters.
2. Need to put more aruco tags on sides and use it to calculate pose and vertices.

---

### 2. calibration_targets
**Description**: A bunch of April tags and aruco based markers and calibration targets in A4 and A3.

---

### 3. corsmal_setup_bringup
**Description**: Package that launches the hardware setup containgin 1xUR16e and 2xAzureKinects. This control layer uses the `twist_controller` to perform servoing. The calibration for the setup right now is being done by detecting a common target from both the cameras. The common frame is the world frame. To calibrate the robot, we looks at the target through a collocated and calibrated camera. Need to change the calibration method to something easy to do and can be done very fast every time we turn on the setup. The launch process should be scalable to multiple devices, so the main driver and process must be  grouped by its functionality. The groups should be 1.Robotic Arm and its calibration, 2.CameraL and its extrinsic calibration, 3.CameraR and its extrinsic calibration, 4. Perception, 5. Robot Behaviour and control.

**Launch process**:
1. `rosmaster_ws` : command that sets the master to be the PC.
2. `launch/azuredk_left.launch` : Launch this on the ws, because the left azure kinect is connected to the ws.
3. `launch/azuredk_right.launch` : Launch this on whichever node is connected to the right camera.
4. `launch/ur16e.launch` : Launch this on the ws, driver for robot.
5. `scripts/extrinsic_publisher_azure_left.py` : publishes the extrinsic of left camera stored in `config/left_azure_extrinsic.json`. This file is obtained by running the calibration script.
6. `scripts/extrinsic_publisher_azure_right.py` : publishes the extrinsic of right camera stored in `config/right_azure_extrinsic.json`. This file is obtained by running the calibration script.
7. `scripts/calibration_publisher_robot.py` : publishes the world to robot transformation.


**Important Scripts**: 
1. `./scripts/extrinsic_calibration_azure_left.py` : a pretty crude way to calibration the left camera and save it to a config file. This uses the "handeye_target" frame detected by the moveit_calibtration method of an aruco grid.
2. `./scripts/extrinsic_calibration_azure_right.py` : a pretty crude way to calibration the right camera and save it to a config file. This uses the "handeye_target" frame detected by the moveit_calibtration method of an aruco grid.
3. `./scripts/get_transformation.py` :get the transformation between any two connected frames.

**Issues**:
1. Calibration must be improved.
2. Too many processes.

---

### 4. grasp_control_actions
**Description**: This hosts the Action server for all the robot control actions that is intended to be used with the final behaviour. The list of actions would be yoink, rest, radial tracking and place. Interruption should be supported for the behaviour to be integrated properly. Currently this is tested in the pose_controller simulatin provided in the grasp_controller_simulator package. The output of the servo type actions like radial tracking and yoink, only interface change with the actual robot should be the velocity topic.

**Actions servers**:
1. `scripts/rest_as.py` : This should infer the rest joint states as a ros parameter, plan a safe path to that joint state and stay there. Action type is infinite execution and motion planning type. Config for this is in the `/config/rest.yaml`.
2. `scripts/place_as.py` : This should place the object in its end effector safely at the right place inferred by an input parameter, default place position must be configured as a ros parameter  if no parameter is provided in the action goal. The Action type is finite execution and motion planning type.Config for this is in the `/config/place.yaml`.
3. `scripts/radial_track_as.py` : This tracks the object provided certain conditions come true like if the object is predefined area of operation and feasible to grasp.  Action type is infinite execution and servo type. Config for this is in the `/config/radial_tracking.yaml`.
4. `scripts/yoink_as.py` : This yoinks the object from the users hand. Action type is finite execution and servo type. Config for this is in the `/config/yoink.yaml`.

These are simulated using the pose_controller type plant.

**Launch Process**:
1. `roscore`
2. `python3 $(rospack find grasp_controller_simulator)/scripts/pose_controller.py`
3. `rosrun tf static_publisher 0 0 0 0 0 0 map world 50`
4. `rostopic pub /filtered_grasp_pose geometry_msgs/PoseStamped <required_target_pose> -r 30`
5. The servo type node that needs to be tested.

### 5. grasp_controller_simulator
**Description**: A very simple physics model to simulate 3d Pose control using velocity. All components are in the "world" frame.

**Scripts** : 
1. `/scripts/graph_err_vel.py`:Subscribes to 2 topic and creates a live plot, very useful for pid tuning.
2. `/scripts/pose_controller.py`: Acts as a plant for 3d pose control using velocity. 
   1. Input Velocity Setpoint topic : `/pose_controller/setpoint_velocity`.
   2. Output Plant Velocity topic : `/pose_controller/current_velocity`.
   3. Output Plant Pose topic : `/pose_controller/current_pose`.

**Launch Process for the simulation**:
1. `roscore`
2. `python3 $(rospack find grasp_controller_simulator)/scripts/pose_controller.py`
3. `rosrun tf static_publisher 0 0 0 0 0 0 map world 50`
4. `rostopic pub /filtered_grasp_pose geometry_msgs/PoseStamped <required_target_pose> -r 30`

### gripper_driver

**Description**: ROS driver for gripper action, supports 2x13 channels, 1 channel for one Digital Input Output, 13 channels allotted for one gripper, 2 grippers can be controlled. `/gripper1` and `/gripper2` are the service namespaces.

The `/arduino/controller/controller.ino` is the file to be loaded into an **arduino mega**. Even numbered pins from \[22,46] are the pins for gripper 1. Odd numbered pins from \[23,47] are the pins for gripper 2.

**Launch Process**:
1. launch `/launch/gripper_driver.launch`.

### robot_setup_sim_bringup

**Description**: Launches a simulation in gazebo of a ur5e, 2 realsense d435i with controllers and moveit move_group. Launches moveit_servo for velocity control of the end effector.

**Launch Process**:
1. launch `/launch/gazebo_bringup.launch`.


