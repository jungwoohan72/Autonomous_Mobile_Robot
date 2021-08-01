<img width="80%" src="https://user-images.githubusercontent.com/45442859/127759452-94709305-5444-4a5b-89e4-6a7230c2ad4b.mp4"/>

1. describe how to install dependent packages needed to run your code

		0) extract the zip file

		1) build the workspace

		cd capstone_1_ROS_gazebo && catkin_make





2. describe how to run the code

		1) launch the map

		roslaunch map_generate import_world.launch

		2 spawn the robot

		roslaunch cheetos_description spawn_cheetos.launch

		3) launch the controller

		roslaunch cheetos_control cheetos_control.launch

		4) run the line tracing node

		rosrun line_tracing line_tracing_node

		5) run the ball detection node

		rosrun ball_detection ball_detect_node

		6) run data integrate node

		rosrun data_integrate data_integration_node





3. Settings and Description of Topics for evalutation

		1) imu topic : set the topic name as '/imu_test'

		2) actuator state topic : list the actuator state topics as below
		* you do not have to include suspension joints

		example)
		a) '/cheetos/front_left_wheel_velocity_controller/state'
		b) '/cheetos/front_right_wheel_velocity_controller/state'
