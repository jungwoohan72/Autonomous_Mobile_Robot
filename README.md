# ME400 Capstone Design

## Objective
1. Overcome the bumps and the slope by tracing black center line in the entrance zone.
2. Collect the blue balls only without colliding with the obstacles in the ball collecting region.
3. Dump the collected blue balls in the designated dumping zone where a green ball is located. 

## Demo Video
![Demo (1)](https://user-images.githubusercontent.com/45442859/127759573-3a3c0d8c-7307-4429-a844-562d636f98fd.gif)

## How to install our code?
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
