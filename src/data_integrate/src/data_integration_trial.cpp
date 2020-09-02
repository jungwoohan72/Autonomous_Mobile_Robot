#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <boost/thread.hpp>
#include <cmath>
#include <vector>


#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "opencv2/opencv.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

boost::mutex map_mutex;

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];
float lidar_obs;

float blue_ball_distance[20];
int near_ball;

int action;

int len;
int n;

int total_ball_num;
float ball_X[20];
float ball_Y[20];

int remaining_blue_ball_num = 3;
int detected_blue_ball_num;
float blue_ball_X[20];
float blue_ball_Y[20];

float angle_min;
float angle_max;
float angle_increment;
std::vector<float> range;

// size of range vector
float angle_temp;

float angular_v_x;
float angular_v_y;
float angular_v_z;

int obstacle_front;
int obstacle_side_back;
int obstacle_side_front;
float obstacle_on_right;
float obstacle_on_left;

//#define RAD2DEG(x) ((x)*180./M_PI)

ros::Publisher pub_left_wheel;
ros::Publisher pub_right_wheel;

std_msgs::Float64 left_wheel_command;
std_msgs::Float64 right_wheel_command;

struct Wheel_commands {
	std_msgs::Float64 command_l;
	std_msgs::Float64 command_r;
};

void upper_camera_Callback(const core_msgs::ball_position::ConstPtr& position_blue){

	int count = position_blue->size;
	detected_blue_ball_num = count;

	std::cout << "Number of remaining blue balls: " << remaining_blue_ball_num << std::endl;
	std::cout << "Number of detected blue balls: " << detected_blue_ball_num << std::endl;


	for(int i = 0; i < detected_blue_ball_num; i++) {
		blue_ball_X[i] = position_blue -> img_x[i];
		blue_ball_Y[i] = position_blue -> img_y[i];
	}

}
/*
void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	map_mutex.lock();

	int count = scan->angle_max / scan->angle_increment;
    lidar_size=count;
    for(int i = 0; i < count; i++)
    {
        lidar_degree[i] = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        lidar_distance[i]=scan->ranges[i];

    }
	map_mutex.unlock();

}
*/
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 msg_cloud;

    // angle in radian
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_increment = scan->angle_increment;
    range = scan->ranges;

    // size of range vector
    lidar_size = range.size();

		obstacle_on_right = 0;
		obstacle_on_left = 0;

		// initializae pointcloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new::pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inf (new::pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_1 (new::pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_front (new::pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_3 (new::pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_side_front (new::pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_side_back (new::pcl::PointCloud<pcl::PointXYZ>);

		// Create the extract object for removal of infinite ditance points
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		pcl::PointIndices::Ptr inf_points(new pcl::PointIndices());

		cloud->is_dense = false;
		cloud->width = lidar_size;
		cloud->height = 1;
		cloud->points.resize(lidar_size);
		cloud_inf->is_dense = false;
		cloud_inf->width = lidar_size;
		cloud_inf->height = 1;
		cloud_inf->points.resize(lidar_size);

		// fill the pointcloud
		for(int i = 0; i < lidar_size; i++){
				angle_temp = angle_min + i*angle_increment;
				if (std::isinf(range[i])==false){

						cloud->points[i].x = range[i]*cos(angle_temp);
						cloud->points[i].y = range[i]*sin(angle_temp);
						cloud->points[i].z = 0;
						cloud_inf->points[i].x = range[i]*cos(angle_temp);
						cloud_inf->points[i].y = range[i]*sin(angle_temp);
						cloud_inf->points[i].z = 0;
				}
				else{
						// indices of infinite distance points
						cloud_inf->points[i].x = 3.5*cos(angle_temp);
						cloud_inf->points[i].y = 3.5*sin(angle_temp);
						cloud_inf->points[i].z = 0;
						inf_points->indices.push_back(i);
				}
		}

		// Remove infinite distance points from cloud
		extract.setInputCloud(cloud);
		extract.setIndices(inf_points);
		extract.setNegative(true);
		extract.filter(*cloud);

		/// 2. Apply passthrough filter to current pointcloud data
		// Create the filtering object

		pcl::PassThrough<pcl::PointXYZ> pass1;
		pass1.setInputCloud (cloud);
		pass1.setFilterFieldName ("x");
		pass1.setFilterLimits (0, 0.6);
		pass1.filter(*cloud_filtered_1);

		pcl::PassThrough<pcl::PointXYZ> pass2;
		pass2.setInputCloud (cloud_filtered_1);
		pass2.setFilterFieldName ("y");
		pass2.setFilterLimits (-0.15, 0.15);
		pass2.filter(*cloud_front);

		pcl::PassThrough<pcl::PointXYZ> pass3;
		pass3.setInputCloud (cloud);
		pass3.setFilterFieldName ("y");
		pass3.setFilterLimits (-0.50, 0.50);
		pass3.filter(*cloud_filtered_3);

		pcl::PassThrough<pcl::PointXYZ> pass4;
		pass4.setInputCloud (cloud_filtered_3);
		pass4.setFilterFieldName ("x");
		pass4.setFilterLimits (0.10, 0.30);
		pass4.filter(*cloud_side_front);

		pcl::PassThrough<pcl::PointXYZ> pass5;
		pass5.setInputCloud (cloud_filtered_3);
		pass5.setFilterFieldName ("x");
		pass5.setFilterLimits (-0.30, -0.10);
		pass5.filter(*cloud_side_back);

		// Coonvert PCL type to sensor_msgs/PointCloud2 type
		obstacle_front = cloud_front->points.size();
		obstacle_side_front = cloud_side_front->points.size();
		obstacle_side_back = cloud_side_back->points.size();

		for (int i = 0; i < cloud_inf->points.size(); i++){
			if (cloud_inf->points[i].y < 0 && cloud_inf->points[i].x > 0) {
				obstacle_on_right += std::abs(cloud_inf->points[i].y);
			}
			else if (cloud_inf->points[i].y > 0 && cloud_inf->points[i].x > 0) {
				obstacle_on_left += std::abs(cloud_inf->points[i].y);
			}
		}

		std::cout << "Front:" << obstacle_front << "    Side back: " << obstacle_side_back << "    Side front: " << obstacle_side_front<<std::endl;
		std::cout << "Left:" << obstacle_on_left << "    Right: " << obstacle_on_right << std::endl;

		//Code to prevent infinite loop where obstacle_on_right and obstacle_on_left are similar.
		if (std::abs(obstacle_on_right-obstacle_on_right)<30) {
			obstacle_on_right = 1;
			obstacle_on_left = 0;
		}
		// Free memory
		cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_inf.reset(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_filtered_1.reset(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_front.reset(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_filtered_3.reset(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_side_front.reset(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_side_back.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void obstacle_avoidance() {

	if (obstacle_on_right > obstacle_on_left) { //Turn right
		left_wheel_command.data = -2.0f;
		right_wheel_command.data = 2.0f;

		pub_left_wheel.publish(left_wheel_command);
		pub_right_wheel.publish(right_wheel_command);
	}
	else { //Turn left
		left_wheel_command.data = 2.0f;
		right_wheel_command.data = -2.0f;

		pub_left_wheel.publish(left_wheel_command);
		pub_right_wheel.publish(right_wheel_command);
	}
}

void check_obstacle_and_avoid() {

  if (obstacle_front != 0){
	  obstacle_avoidance();
	}
	else {
		if (obstacle_side_back < 15) {
			left_wheel_command.data = 5.0f;
			right_wheel_command.data = 5.0f;

			pub_left_wheel.publish(left_wheel_command);
			pub_right_wheel.publish(right_wheel_command);
		}
		else {
			left_wheel_command.data = 0.0f;
			right_wheel_command.data = 0.0f;

			pub_left_wheel.publish(left_wheel_command);
			pub_right_wheel.publish(right_wheel_command);
		}
	}
}

//Rotating until all the remaining blue balls are detected
bool rotate_to_detect() {
	std::cout<<"Rotating…"<<std::endl;
	std_msgs::Float64 left_wheel_command;
	std_msgs::Float64 right_wheel_command;

	if (remaining_blue_ball_num == detected_blue_ball_num) {
		left_wheel_command.data = 0;
		right_wheel_command.data = 0;

		pub_left_wheel.publish(left_wheel_command);
		pub_right_wheel.publish(right_wheel_command);
		return true;
	}

	else {
		left_wheel_command.data = -4;
		right_wheel_command.data = 4;

		pub_left_wheel.publish(left_wheel_command);
		pub_right_wheel.publish(right_wheel_command);
		return false;
	}
}

//Align the robot with a ball and collect the ball
Wheel_commands foobar(float x, float target) {
	std::cout<<"closest ball x:"<<std::fixed<<x<<std::endl;
	Wheel_commands result;
	if(x == 0.0f) {
		result.command_l.data = -(2.0f);
		result.command_r.data = 2.0f;
		return result;
	}
	x = x > 1.0f? 1.0f : x;
	x = x < -1.0f? -1.0f : x;
	float direction_coeff = 5.0f;
	float linear_coeff = 5.0f;
	result.command_l.data = (direction_coeff* x + linear_coeff);
	result.command_r.data = (-direction_coeff* x + linear_coeff);
	return result;
}

Wheel_commands collect_ball_update() {
	//Find a blue ball with the maximum Y value
	float min_y = 10.0f;
	float min_y_x = 0.0f;
	for(int i = 0; i < detected_blue_ball_num; i++) {
		if(blue_ball_Y[i] < min_y) {
			min_y = blue_ball_Y[i];
			min_y_x = blue_ball_X[i];
		}
	}
	return foobar(min_y_x, 0.0f);
}

void imu_Callback(const sensor_msgs::Imu::ConstPtr &imu) {
	angular_v_x = imu->angular_velocity.x;
	angular_v_y = imu->angular_velocity.y;
	angular_v_z = imu->angular_velocity.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_integation");
	ros::NodeHandle nh;
	ros::Subscriber sub_upper_camera = nh.subscribe<core_msgs::ball_position>("/position_blue", 1000, upper_camera_Callback);
	ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("/imu", 1000, imu_Callback);
	ros::Subscriber sub_lidar = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_cb);
	pub_left_wheel= nh.advertise<std_msgs::Float64>("/cheetos/front_left_wheel_velocity_controller/command", 10);
	pub_right_wheel= nh.advertise<std_msgs::Float64>("/cheetos/front_right_wheel_velocity_controller/command", 10);

	ros::Rate loop_rate(50);
  /*
	std::cout<<"Moving robot…"<<std::endl;
	bool found_all_blue_balls = false;
	while(ros::ok() && !found_all_blue_balls){
		ros::spinOnce();
		loop_rate.sleep();
		found_all_blue_balls = rotate_to_detect();
	}
  // Rotate until the robot detects all the blue balls in the map.
	std::cout<<"collecting balls"<<std::endl;
	std::cout.precision(8);
	*/
	while (ros::ok) {
		ros::spinOnce();
		/*
		ros::spinOnce();
		auto commands = collect_ball_update();

		pub_left_wheel.publish(commands.command_l);
		pub_right_wheel.publish(commands.command_r);
		std::cout<<"L:"<<std::fixed<<commands.command_l.data<<", R:"<<std::fixed<<commands.command_r.data<<std::endl;
		*/
	  if (obstacle_front != 0) {
		  //Fisrt avoid collision
			while (ros::ok) {
				ros::spinOnce();
				loop_rate.sleep();

				check_obstacle_and_avoid();

				if (left_wheel_command.data == 0.0f && right_wheel_command.data == 0.0f){
					break;
				}
			}
			//Then check if it is wall or obstacle. If obstacle, modify the route, else stay still.
			if (obstacle_side_back+obstacle_side_front < 30) {
				if (obstacle_on_left > obstacle_on_right) {
					while (obstacle_side_front < 15) {
						ros::spinOnce();
						loop_rate.sleep();

						left_wheel_command.data = -2.0f;
						right_wheel_command.data = 2.0f;

						pub_left_wheel.publish(left_wheel_command);
						pub_right_wheel.publish(right_wheel_command);
					}
					left_wheel_command.data = 0.0f;
					right_wheel_command.data = 0.0f;

					pub_left_wheel.publish(left_wheel_command);
					pub_right_wheel.publish(right_wheel_command);
			  }
				else {
					while (obstacle_side_front < 15) {
						ros::spinOnce();

						left_wheel_command.data = 2.0f;
						right_wheel_command.data = -2.0f;

						pub_left_wheel.publish(left_wheel_command);
						pub_right_wheel.publish(right_wheel_command);

						loop_rate.sleep();
				  }
					left_wheel_command.data = 0.0f;
					right_wheel_command.data = 0.0f;

					pub_left_wheel.publish(left_wheel_command);
					pub_right_wheel.publish(right_wheel_command);
				}
			}
		}
	  loop_rate.sleep();
  }
    //remaining_blue_ball_num = 3;

  return 0;
}
