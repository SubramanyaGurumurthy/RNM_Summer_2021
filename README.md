# RNM_Summer_2021

In this project, we developed an image guided robotic needle placement system using the Robot Operating System (ROS) framework. We used a depth camera (Kinect Azure by Microsoft) mounted on a robot arm (Panda by Franka Emika) to record 3D images of a chest phantom. We found the transformation between the robot's ende effector and the camera with an eye-in-hand calibration. Using this transformation we were able to stitch the individual images to a combined scan while the robot drives the camera around the phantom. We then registered this scan to a high resolution model, obtained from computer tomography. Within the high resolution model a target for the needle will be given. We performed  trajectory planning to find a collision free and kinematically feasible path to the target. Lastly, we exchanged the camera for a needle (mock-up) and let the robot perform the insertion.

## Dependencies:

   * Ros - Melodic
   * python version - 2.7


	1.Python Libraries used:
		numpy - version: 1.16.6
		opencv - version: 3.2.0
		scipy - version: 1.2.3
		open3d - version: 0.9.0.0

	2.Ros library:
		cv_bridge



