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
**Description**: Package to simulate the 6Dof perception using aruco tags on boxes with hardcoded dimensions. configurable input rgb image, depth image and camera info topic which can be used to easily interface it with different cameras. Performs infinite impulse repose linear filtering to smoothen the estimated pose. Uses spherical linear interpolation for filtering the pose. Supports multiple patterns and pattern transform, but this is not configurable yet. 

**Todo lists**
- [ ] This needs to be more accurate by having patterns on opposite sides with april tags instead of aruco tags, should be interfaced with two cameras with time synchronization. So that the pose is detected without any interruptions.
- [ ] The pattern on each side should be 2x2 ideally.

**Important Scripts**: 
1. `./scripts/create_marker.py` : creates a series of markers on an A4 sheet.
2. `./scripts/aruco_proxy_pose_extimation_filtered.py` This is the latest aruco proxy pose estimation script, uses a few on the fly config along with the the config file in `./config/aruco_proxy_pose_estimation.yaml`.
3. `pose_evaluation.py` : this just gets the mean, mode, median, std_dev and variance of a measured pose.

**Launch process**:
1. `./launch/aruco_proxy_pose_estimation_realsense.launch` : launches the node for realsense based cameras, this launches the realsense node too.
2. `./launch/aruco_proxy_pose_estimation.launch` : launches the node for azure kinect based cameras, uses only the right camera.
---

### 2. calibration_targets
**Description**: A bunch of April tags and aruco based markers and calibration targets in A4 and A3.

---

### 3. corsmal_setup_bringup
**Description**: Package that launches the hardware setup containgin 1xUR16e and 2xAzureKinects. This control layer uses the `twist_controller` to perform servoing. The `twist_controller` is speculated to be more efficient than the moveit servo route. Note that the `twist_controller` controllers the configured end effector with respect to the `base` frame of the robot.

To use moveit servo, use the `joint_group_vel_controller` with the moveit servo node. This seems to have the best performance over using the `joint_group_pos_controller`. But its still suboptimal compared to the `twist_controller`, this is the best option for servo options for UR robots.

The calibration for the setup right now is being done by detecting a common target from both the cameras. The common frame is the world frame. To calibrate the robot, we looks at the target through a collocated and calibrated camera.

**Control Integration**:
The parent frame of the whole system is `world` but the planning frame must be `base_link` for ease of configuration. The positions of the object pose must be with respect to `base_link` due to moveit constraints.

**TODO list**
- [ ] Need to change the calibration method to something easy to do and can be done very fast every time we turn on the setup.
- [ ] The launch process should be scalable to multiple devices, so the main driver and process must be  grouped by its functionality. The groups should be 
  1. Robotic Arm and its calibration, 
  2. CameraL and its extrinsic calibration 
  3. CameraR and its extrinsic calibration
  4. Perception
  5. Robot Behaviour and control

**Current Calibration Procedure**
Use the `moveit_calibration` package to detect the target, the target will be called `handeye_target` once it is detected by the camera. The tf of the `handeye_target` will be published to the camera_info's topic?? need to check how moveit_calibration is done.

Once the target is detected by the camera, run the `scripts/extrinsic_calibration_azure_left.py` or `scripts/extrinsic_calibration_azure_right.py` depending on the camera that needs to be calibrated.

To calibrate the robot, attach a camera onto the end effector, do that cameras calibration using the static target, once the calibration is done, run the `scripts/get_transformation.py` to get the calibration of handeye_target to base_link and save it in the config folder in a file called `config/calibration_robot.json`. The `scripts/calibration_publisher_robot.py` gets the calibration from this file and publishes a static transform.

**Launch process**:
1. `rosmaster_ws` : command that sets the master to be the PC, this should be done in every new terminal.
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

---

### 4. grasp_control_actions
**Description**: This hosts the Action server for all the robot control actions that is intended to be used with the final behaviour. The list of actions would be yoink, rest, radial tracking and place.</br>
Supported Features(Tentative):</br>
- [x] Interruptible actions.
- [x] Configurable.
- [x] No source code change between real and sim.
- [ ] On the fly configuration.
- [ ] Error handling in case of loss of pose.
- [ ] Modularity and reusability, not true since the control and policy are tightly coupled in the context of actions, and the velocity filter is tightly coupled to the context of using the `twist_controller` because of the transformation.

**Actions servers**:
1. `scripts/rest_as.py` : This should infer the rest joint states as a ros parameter, plan a safe path to that joint state and stay there. Action type is infinite execution and motion planning type. Config for this is in the `/config/rest.yaml`.
2. `scripts/place_as.py` : This should place the object in its end effector safely at the right place inferred by an input parameter, default place position must be configured as a ros parameter  if no parameter is provided in the action goal. The Action type is finite execution and motion planning type.Config for this is in the `/config/place.yaml`.
3. `scripts/place_vel_as.py` : The configuration follows the same rules as classic place, but the control is done using PID with servoing instead of trajectory planning and execution. The Action type is finite execution and motion planning type.Config for this is in the `/config/place.yaml`.
4. `scripts/radial_track_as.py` : This tracks the object provided certain conditions come true like if the object is predefined area of operation and feasible to grasp.  Action type is infinite execution and servo type. Config for this is in the `/config/radial_tracking.yaml`.
5. `scripts/yoink_as.py` : This yoinks the object from the users hand. Action type is finite execution and servo type. Config for this is in the `/config/yoink.yaml`.
6. `scripts/client.py` : Client/ high level behaviour. 

These are tested with the real robot setup.

**Launch Process**:
1. Sim/Real Launch process.
2. for simulation `launch/action_server_sim.launch` loads the configuration files meant for simulation. for real `launch/action_servers.launch`.

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
This loads the following controllers : 
- `joint_group_pos_controller` : used by moveit servo for servo control of the end effector. In stopped state, need to start it using command `/contoller_manager/switch_controller` to start `/joint_group_pos_controller` and stop `pos_joint_traj_controller`.
- `joint_state_controller` : joint state broadcast
- `pos_joint_traj_controller` : joint trajectory control for moveit planning, need to switch back to this for moveit planning layer to take control. 
 

The simulation starts with a paused physics, to unpause it, we need to use the services : `/gazebo/pause_physics` and `/gazebo/unpause_physics`.


**Launch Process**:
1. launch `/launch/gazebo_bringup.launch`.


