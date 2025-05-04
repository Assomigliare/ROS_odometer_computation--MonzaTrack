# ROS_odometer_computation--MonzaTrack
It is a basic project of ROS to compute the odometry and vehicle motion in a real world. The first node of the trajetory was not always perfect because of incorrected steering factor and datas, but we could obtain an ideal track in the second node. Additionally, it is also interesting to fine-tune those parameters by referencing them in GPS.

Project Overview:
It is the Monza track -- You can find it through Google Map

	odometer node:
	- This node aims to calculate positions according to the given velocity and steering angle.
	- Paramaters:
		STEERING_FACTOR = 43 // it is not a precise number
		bias = 9.5 degrees // project.bag always exists significant deviations
	- We removed small steering bias when the vehicle travels straight and fine-tuned the steering factor, and used a bicycle model combined with Ackermann Steering Geometry to estimate vehicle position.

	gps_odometer node:
	- We can get latitude, longitude and altitude data to compute positions. (ECEF and ENU)
	- GPS datas were always correct and we filtered out the data points where GPS positions were lost.
   
	sector_times node:
	- Checkpoints capture (coordinates according to Google Map):
		1st checkpoint: [45.630116, 9.289851]
		2nd checkpoint: [45.623546, 9.287241]
		3rd checkpoint: [45.6189323717, 9.2811788719] -->initial position
	- This node monitors the vehicle's progression through a sequence of GPS checkpoints.
	- When the vehicle reaches within 5 meters of a checkpoint, it records the sector's elapsed time and average speed.


Custom messages:
	  int8 current_sector
    float64 current_sector_time
    float64 current_sector_mean_speed

Stratup command:
    roslaunch first_project launch.launch

  Alternatively, nodes can be run respectively:
	  rosrun first_project odometer
    rosrun first_project gps_to_odom
    rosrun first_project sector_times
