# RNM_Summer_2021

In this project, we developed an image guided robotic needle placement system using the Robot Operating System (ROS) framework. We used a depth camera (Kinect Azure by Microsoft) mounted on a robot arm (Panda by Franka Emika) to record 3D images of a chest phantom. We found the transformation between the robot's ende effector and the camera with an eye-in-hand calibration. Using this transformation we were able to stitch the individual images to a combined scan while the robot drives the camera around the phantom. We then registered this scan to a high resolution model, obtained from computer tomography. Within the high resolution model a target for the needle will be given. We performed  trajectory planning to find a collision free and kinematically feasible path to the target. Lastly, we exchanged the camera for a needle (mock-up) and let the robot perform the insertion.

## Dependencies:

* Ros - Melodic
* python version - 2.7


1. Python Libraries used:
* numpy - version: 1.16.6
* opencv - version: 3.2.0
* scipy - version: 1.2.3
* open3d - version: 0.9.0.0

2. Ros library:
* cv_bridge

## Folder structure:
The project contains different nodes which are basically the packages. Each node/package contains src/scripts folder, launch folder, srv folder.
	
1. Src/Scripts folder: This folder contains the main executable python files - ".py" extension files.
2. launch folder: this folder contains the launch files, which launches the node and completes relevant operation.
3. srv folder: this folder usually contains the relevant type of service which is required for the corresponding nodes operations.

For more details about the folder structures of ROS please refer to this [link](http://wiki.ros.org/Packages)

Kinematics:
	The packages related to kinematics can be found at "/catkin_ws/src/franka_ros-simulation/" folder with suffix "z_" i.e., 
	1. z_forward_kinematics
	2. z_inverse_kinematics
	3. z_trajectory_execution
	4. z_trajectory_planning


vision: 
	The packages related to vision can be found at "/catkin_ws/src/" which are:
	1. camera_calib
	2. hand_eye
	3. model_registration

## Specific instructions to launch files:
	
1. Camera calibration node:
	launch "trajectory_camera_calibration.launch" or "trajectory_camera_calibration_sim.launch" file before launching camera_calib.launch
2. Hand eye calibration:
	launch "trajectory_hand_eye_calibration.launch" "trajectory_hand_eye_calibration_sim.launch" file before launching hand_eye.launch

Since the master computer did not have compatibility with point cloud libraries, the launch of point cloud and "trajectory_target_execution.launch" or "trajectory_target_execution_sim.launch" is 
not automated.

### Some useful terminal commands that can be used to launch nodes if this project has been integrated into a ROS environment:
	
	* while testing on real robot:
		$ roslaunch z_forward_kinematics forward_kinematics_test.launch
		$ roslaunch z_inverse_kinematics inverse_kinematics.launch
		$ roslaunch z_trajectory_planning trajectory_camera_calibration.launch
		$ roslaunch z_trajectory_planning trajectory_hand_eye_calibration.launch
		$ roslaunch z_trajectory_planning trajectory_planning_test.launch
		$ roslaunch z_trajectory_planning trajectory_pointcloud_registration.launch
		$ roslaunch z_trajectory_planning trajectory_target_execution.launch


	* While testing on simulator:
		$ roslaunch z_forward_kinematics forward_kinematics_test.launch
		$ roslaunch z_inverse_kinematics inverse_kinematics.launch
		$ roslaunch z_trajectory_planning trajectory_camera_calibration_sim.launch
		$ roslaunch z_trajectory_planning trajectory_hand_eye_calibration_sim.launch
		$ roslaunch z_trajectory_planning trajectory_planning_test_sim.launch
		$ roslaunch z_trajectory_planning trajectory_pointcloud_registration_sim.launch
		$ roslaunch z_trajectory_planning trajectory_target_execution_sim.launch

### Link to the working video can be found at:
Link: https://cloud.tuhh.de/index.php/s/CrF3tJNKX4QNimt
