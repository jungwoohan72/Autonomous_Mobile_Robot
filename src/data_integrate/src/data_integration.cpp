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
#include "core_msgs/ball_position_z.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

#include "opencv2/opencv.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include "data_integration.h"
#include "robot_position.h"

boost::mutex map_mutex;

float lidar_closest_front_degree;
float lidar_closest_front_distance;

std::vector<float> blue_ball_distance(20);
int near_ball;

int action;

int len;
int n;

int total_ball_num;
std::vector<float> ball_X(20);
std::vector<float> ball_Y(20);

int remaining_blue_ball_num = 3;
int third_ball_count = 0;

std::vector<float> lower_blue_ball_Z(20);

int detected_lower_red_ball_num;
std::vector<float> lower_red_ball_X(20);
std::vector<float> lower_red_ball_Y(20);
std::vector<float> lower_red_ball_Z(20);

int blue_ball_catching_countdown = 0;

float angle_min;
float angle_max;
float angle_increment;
std::vector<float> range(10);

// size of range vector
float angle_temp;

float angular_v_x;
float angular_v_y;
float angular_v_z;
float linear_a_x;
float linear_a_y;
float linear_a_z;

int obstacle_front;
int obstacle_side_back;
int obstacle_side_front;
int obstacle_back_surrounding;
float obstacle_on_right;
float obstacle_on_left;

int CCW = 1;
int CW = -1;
int next_spin_direction = CCW;

#define loop_frequency (20)
//PID Gains
float p = 5.5f;
float d = p*0.5f / (float)loop_frequency;
float i = p*0.002f * (float)loop_frequency;

#define ARBITRARY_NAN 999.0f

ros::Publisher pub_left_wheel;
ros::Publisher pub_right_wheel;

std_msgs::Float64 left_wheel_command;
std_msgs::Float64 right_wheel_command;

struct Wheel_commands {
	std_msgs::Float64 command_l; //왼쪽 바퀴의 모터 인풋
	std_msgs::Float64 command_r; //오른쪽 바퀴의 모터 인풋
};

std::vector<Point> MOVE_QUEUE;

void position_blue_upper_Callback(const core_msgs::ball_position::ConstPtr& position_blue_upper){

	int count = position_blue_upper->size;
	detected_blue_ball_num = count;

	//std::cout << "Number of remaining blue balls: " << remaining_blue_ball_num << std::endl;
	//std::cout << "Number of detected blue balls: " << detected_blue_ball_num << std::endl;

	blue_ball_X.assign(position_blue_upper -> img_x.begin(), position_blue_upper -> img_x.end());
	blue_ball_Y.assign(position_blue_upper -> img_y.begin(), position_blue_upper -> img_y.end());
}

void position_blue_rear_Callback(const core_msgs::ball_position::ConstPtr& position_blue_rear){
	int count = position_blue_rear->size;
	detected_rear_blue_ball_num = count;

	rear_blue_ball_X.assign(position_blue_rear -> img_x.begin(), position_blue_rear -> img_x.end());
	rear_blue_ball_Y.assign(position_blue_rear -> img_y.begin(), position_blue_rear -> img_y.end());
	blue_position_update();
}

void position_rear_up_blue_Callback(const core_msgs::ball_position::ConstPtr& position_rear_up_blue){
	int count = position_rear_up_blue->size;
	detected_rear_up_blue_ball_num = count;

	rear_up_blue_ball_X.assign(position_rear_up_blue -> img_x.begin(), position_rear_up_blue -> img_x.end());
	rear_up_blue_ball_Y.assign(position_rear_up_blue -> img_y.begin(), position_rear_up_blue -> img_y.end());

	blue_position_update();
}

void position_blue_lower_Callback(const core_msgs::ball_position_z::ConstPtr& position_blue_lower){
	int count = position_blue_lower->size;
	detected_lower_blue_ball_num = count;
	lower_blue_ball_X.assign(position_blue_lower -> img_x.begin(), position_blue_lower -> img_x.end());
	lower_blue_ball_Y.assign(position_blue_lower -> img_y.begin(), position_blue_lower -> img_y.end());
	lower_blue_ball_Z.assign(position_blue_lower -> img_z.begin(), position_blue_lower -> img_z.end());
}

void position_red_Callback(const core_msgs::ball_position::ConstPtr& position_red){
	int count = position_red->size;
	detected_red_ball_num = count;
	red_ball_X.assign(position_red -> img_x.begin(), position_red -> img_x.end());
	red_ball_Y.assign(position_red -> img_y.begin(), position_red -> img_y.end());
}

void position_red_rear_Callback(const core_msgs::ball_position::ConstPtr& position_red_rear){
	int count = position_red_rear->size;
	detected_rear_red_ball_num = count;
	rear_red_ball_X.assign(position_red_rear -> img_x.begin(), position_red_rear -> img_x.end());
	rear_red_ball_Y.assign(position_red_rear -> img_y.begin(), position_red_rear -> img_y.end());
	obstacle_position_update();
}

void position_rear_up_red_Callback(const core_msgs::ball_position::ConstPtr& position_rear_up_red){
	int count = position_rear_up_red->size;
	detected_rear_up_red_ball_num = count;

	rear_up_red_ball_X.assign(position_rear_up_red -> img_x.begin(), position_rear_up_red -> img_x.end());
	rear_up_red_ball_Y.assign(position_rear_up_red -> img_y.begin(), position_rear_up_red -> img_y.end());

	obstacle_position_update();
}

void position_red_lower_Callback(const core_msgs::ball_position_z::ConstPtr& position_red_lower){
	int count = position_red_lower->size;
	detected_lower_red_ball_num = count;
	lower_red_ball_X.assign(position_red_lower -> img_x.begin(), position_red_lower -> img_x.end());
	lower_red_ball_Y.assign(position_red_lower -> img_y.begin(), position_red_lower -> img_y.end());
	lower_red_ball_Z.assign(position_red_lower -> img_z.begin(), position_red_lower -> img_z.end());
}

void green_ball_Callback(const core_msgs::ball_position::ConstPtr& position_green) {
	detected_green_ball_num = position_green->size;
	green_ball_X.assign(position_green -> img_x.begin(), position_green -> img_x.end());
	green_ball_Y.assign(position_green -> img_y.begin(), position_green -> img_y.end());
}

void rear_up_green_ball_Callback(const core_msgs::ball_position::ConstPtr& position_rear_up_green) {
	detected_rear_up_green_ball_num = position_rear_up_green->size;

	rear_up_green_ball_X.assign(position_rear_up_green -> img_x.begin(), position_rear_up_green -> img_x.end());
	rear_up_green_ball_Y.assign(position_rear_up_green -> img_y.begin(), position_rear_up_green -> img_y.end());
}

void green_ball_Callback_lower(const core_msgs::ball_position_z::ConstPtr& position_green_lower) {
	detected_green_ball_num_lower = position_green_lower->size;
	green_ball_X_lower.assign(position_green_lower -> img_x.begin(), position_green_lower -> img_x.end());
	green_ball_Y_lower.assign(position_green_lower -> img_y.begin(), position_green_lower -> img_y.end());
}

void green_ball_Callback_rear(const core_msgs::ball_position::ConstPtr& position_green_rear) {
	detected_green_ball_num_rear = position_green_rear->size;
	green_ball_X_rear.assign(position_green_rear -> img_x.begin(), position_green_rear -> img_x.end());
	green_ball_Y_rear.assign(position_green_rear -> img_y.begin(), position_green_rear -> img_y.end());
}


void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 msg_cloud;

    // angle in radian
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_increment = scan->angle_increment;
    range = scan->ranges;

    // size of range vector
    lidar_size = range.size();
    lidar_closest_front_distance = 999.0f;
    for(int i = 0; i < range.size(); i++) {
		lidar_degree.at(i) = RAD2DEG(scan->angle_min + scan->angle_increment * i);
		lidar_distance.at(i) = scan->ranges.at(i);
		if((i < range.size() / 4) || (i > range.size() * 3 / 4)) {
			if(lidar_closest_front_distance > lidar_distance.at(i)) {
				lidar_closest_front_distance = lidar_distance.at(i);
				lidar_closest_front_degree = lidar_degree.at(i);
			}
		}
	}
	robot_position_update();

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
	for(int i = 0; i < range.size(); i++){
		angle_temp = angle_min + i*angle_increment;
		if (std::isinf(range.at(i))==false){

			cloud->points.at(i).x = range.at(i)*cos(angle_temp);
			cloud->points.at(i).y = range.at(i)*sin(angle_temp);
			cloud->points.at(i).z = 0;
			cloud_inf->points.at(i).x = range.at(i)*cos(angle_temp);
			cloud_inf->points.at(i).y = range.at(i)*sin(angle_temp);
			cloud_inf->points.at(i).z = 0;
		}
		else{
			// indices of infinite distance points
			cloud_inf->points.at(i).x = 3.5*cos(angle_temp);
			cloud_inf->points.at(i).y = 3.5*sin(angle_temp);
			cloud_inf->points.at(i).z = 0;
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
	pass1.setFilterLimits (0.19, 0.40);
	pass1.filter(*cloud_filtered_1);

	pcl::PassThrough<pcl::PointXYZ> pass2;
	pass2.setInputCloud (cloud_filtered_1);
	pass2.setFilterFieldName ("y");
	pass2.setFilterLimits (-0.16, 0.16);
	pass2.filter(*cloud_front);

	pcl::PassThrough<pcl::PointXYZ> pass3;
	pass3.setInputCloud (cloud);
	pass3.setFilterFieldName ("y");
	pass3.setFilterLimits (-0.50, 0.50);
	pass3.filter(*cloud_filtered_3);

	pcl::PassThrough<pcl::PointXYZ> pass4;
	pass4.setInputCloud (cloud_filtered_3);
	pass4.setFilterFieldName ("x");
	pass4.setFilterLimits (0.10, 0.27);
	pass4.filter(*cloud_side_front);

	pcl::PassThrough<pcl::PointXYZ> pass5;
	pass5.setInputCloud (cloud_filtered_3);
	pass5.setFilterFieldName ("x");
	pass5.setFilterLimits (-0.30, -0.13);
	pass5.filter(*cloud_side_back);

	pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_y_surrounding (new::pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass6;
	pass6.setInputCloud (cloud);
	pass6.setFilterFieldName ("y");
	pass6.setFilterLimits (-0.16, 0.16);
	pass6.filter(*_cloud_y_surrounding);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_back_surrounding (new::pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass7;
	pass7.setInputCloud (_cloud_y_surrounding);
	pass7.setFilterFieldName ("x");
	pass7.setFilterLimits (-0.30, 0.18);
	pass7.filter(*cloud_back_surrounding);

	// Coonvert PCL type to sensor_msgs/PointCloud2 type
	obstacle_front = cloud_front->points.size();
	obstacle_side_front = cloud_side_front->points.size();
	obstacle_side_back = cloud_side_back->points.size();
	obstacle_back_surrounding = cloud_back_surrounding->points.size();

	for (int i = 0; i < cloud_inf->points.size(); i++){
		if (cloud_inf->points.at(i).y < 0 && cloud_inf->points.at(i).x > 0) {
			obstacle_on_right += std::abs(cloud_inf->points.at(i).y);
		}
		else if (cloud_inf->points.at(i).y > 0 && cloud_inf->points.at(i).x > 0) {
			obstacle_on_left += std::abs(cloud_inf->points.at(i).y);
		}
	}

	//std::cout << "Front:" << obstacle_front << "    Side back: " << obstacle_side_back << "    Side front: " << obstacle_side_front<<std::endl;
	//std::cout << "Left:" << obstacle_on_left << "    Right: " << obstacle_on_right << std::endl;

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
	_cloud_y_surrounding.reset(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_back_surrounding.reset(new pcl::PointCloud<pcl::PointXYZ>);
}


void front_obstacle_avoidance() {
	if (lidar_closest_front_degree > 180.0f) { //Turn left
		left_wheel_command.data = -4.0f;
		right_wheel_command.data = 4.0f;
		std::cout << "Avoiding: Turn left\n";
		next_spin_direction *= -1;
	}
	else { //Turn left
		left_wheel_command.data = 4.0f;
		right_wheel_command.data = -4.0f;
		std::cout << "Avoiding: Turn right\n";
		next_spin_direction *= -1;
	}
}

bool check_obstacle_and_avoid() {

  	if (obstacle_front != 0){
	 	front_obstacle_avoidance();
		return true;
	}
	else {
		if (obstacle_back_surrounding > 0) {
			left_wheel_command.data = 4.0f;
			right_wheel_command.data = 4.0f;
			std::cout << "Avoiding: Go forward\n";
			return true;
		}
		else {
			return false;
		}
	}
}

void front_red_ball_avoidance() {
	if (lidar_closest_front_degree > 180.0f) { //Turn left
		left_wheel_command.data = -4.0f;
		right_wheel_command.data = 4.0f;
		std::cout << "Red Ball Avoiding: Turn left\n";
		next_spin_direction *= -1;
	}
	else { //Turn left
		left_wheel_command.data = 4.0f;
		right_wheel_command.data = -4.0f;
		std::cout << "Red Ball Avoiding: Turn right\n";
		next_spin_direction *= -1;
	}
}

bool check_red_ball_and_avoid() {
  int red_ball_front = 0;

	for (int i = 0; i < detected_red_ball_num; i++) {
		if (std::abs(red_ball_X.at(i)) > 0.360f) {
			red_ball_front = 0;
			continue;
		}
		else if (red_ball_Y.at(i) < 0.380f) {
			red_ball_front = 1;
		}
	}

  if (red_ball_front == 1){
	 	front_red_ball_avoidance();
		return true;
	}
	else {
		if (obstacle_back_surrounding > 0) {
			left_wheel_command.data = 4.0f;
			right_wheel_command.data = 4.0f;
			std::cout << "Red Ball Avoiding: Go forward\n";
			return true;
		}
		else {
			return false;
		}
	}
}


//맵 상에 있는 파란공의 갯수를 업데이트 하며 제자리에서 도는 함수.
bool rotate_to_detect() {
	std_msgs::Float64 left_wheel_command; //왼쪽 바퀴에 publish 될 모터 제어 커맨드
	std_msgs::Float64 right_wheel_command; //오른쪽 바퀴에 publish 될 모터 제어 커맨드

  //맵 상에 남아있는 공의 갯수와 모아야 할 공의 갯수가 같다면 회전을 멈춘다.
	if (remaining_blue_ball_num == detected_blue_ball_num) {
		left_wheel_command.data = 0;
		right_wheel_command.data = 0;

		pub_left_wheel.publish(left_wheel_command);
		pub_right_wheel.publish(right_wheel_command);
		return true;
	}
  //collect_ball_update 함수에서 결정된 next_spin_direction에 따라 왼쪽으로 회전할 것인지 오른쪽으로 회전할 것인지 결정한다.
	else {
		left_wheel_command.data = -3.0f * next_spin_direction;
		right_wheel_command.data = 3.0f * next_spin_direction;

		pub_left_wheel.publish(left_wheel_command);
		pub_right_wheel.publish(right_wheel_command);
		return false;
	}
}

inline float saturate(float x, float max, float min) {
	return x > max? max : (x < min? min : x);
}

Wheel_commands motor_control(float horizontal_error, float vertical_error, float& prev_error, float& integrated_error) {
	Wheel_commands result;
	if(prev_error == 0.0f) {
		prev_error = horizontal_error;
	}
	//
	float error_d = horizontal_error - prev_error;
	prev_error = (prev_error + horizontal_error) / 2;

	float INTEGRAL_DECAY = 0.95f;
	integrated_error = INTEGRAL_DECAY * saturate(integrated_error + horizontal_error, 1000.0f, -1000.0f);
	float linear_coeff = 1.0f;

	float linear_vel = linear_coeff * saturate(vertical_error * 2.0f, 3.0f, 1.0f);
	float rotation_vel = p * horizontal_error + d * error_d + i * integrated_error;
	printf("p:%03.0f%% d:%03.0f%% i:%03.0f%%\n", p * horizontal_error / rotation_vel * 100, d * error_d / rotation_vel * 100, i * integrated_error / rotation_vel * 100);
	/*if(std::abs(error) / r > 0.05f) {
		linear_vel = 1.0f;
	}*/
	float angular_vel = rotation_vel / 2.29f;
	result.command_l.data = saturate( (angular_vel * 2.29f) + linear_vel, 5.0f, -5.0f );
	result.command_r.data = saturate( -(angular_vel * 2.29f) + linear_vel, 5.0f, -5.0f);
	return result;
}

float PREVIOUS_ERROR = 0.0f;
float INTEGRAL_ERROR = 0.0f;

//앞-아래 카메라를 사용하여 로봇을 파란 공과 정렬 시키고 모으는 함수
Wheel_commands command_to_align(float r, float x, float target) {
	//std::cout<<"closest ball x:"<<std::fixed<<x<<std::endl;
	Wheel_commands result;
	if(x == ARBITRARY_NAN) { //ARBITRARY_NAN = 999.00f
		PREVIOUS_ERROR = 0.0f;
		INTEGRAL_ERROR = 0.0f;
		if(blue_ball_catching_countdown > 0) { //blue_ball_catching_countdown = 5
			blue_ball_catching_countdown--;
			result.command_l.data = 1* 4.0f;
			result.command_r.data = 1* 4.0f;
			return result;
		}
		if(next_spin_direction == CW) { //시계방향으로 회전하도록 모터 인풋을 줌.
			result.command_l.data = 4.0f;
			result.command_r.data = -4.0f;
			std::cout << "Finding ball: Turn right\n";
		}
		else { //반시계 방향으로 회전하도록 모터 인풋을 줌.
			result.command_l.data = -4.0f;
			result.command_r.data = 4.0f;
			std::cout << "Finding ball: Turn left\n";
		}
		PREVIOUS_ERROR = 0.0f;
		INTEGRAL_ERROR = 0.0f;
		return result;
	}

	float error = target - x;
	result = motor_control(error, r, PREVIOUS_ERROR, INTEGRAL_ERROR);
	return result;
}

Wheel_commands command_to_align_rear(float r, float x, float target) {
	//std::cout<<"closest ball x:"<<std::fixed<<x<<std::endl;
	Wheel_commands result;
	if(x == ARBITRARY_NAN) {
		PREVIOUS_ERROR = 0.0f;
		INTEGRAL_ERROR = 0.0f;
		if(next_spin_direction == CW) {
			result.command_l.data = 4.0f;
			result.command_r.data = -4.0f;
			std::cout << "Finding ball: Turn right\n";
		}
		else {
			result.command_l.data = -4.0f;
			result.command_r.data = 4.0f;
			std::cout << "Finding ball: Turn left\n";
		}
		PREVIOUS_ERROR = 0.0f;
		INTEGRAL_ERROR = 0.0f;
		return result;
	}
	blue_ball_catching_countdown = 5;

	float error = target - x;
	result = motor_control(error, r, PREVIOUS_ERROR, INTEGRAL_ERROR);
	return result;
}


Wheel_commands collect_ball_update() {
	//Find a blue ball with the maximum Y value
	float min_dist = 100.0f;
	float min_dist_x = 0.0f;
	for(int i = 0; i < blue_ball_X.size(); i++) {
		float ball_dist = std::sqrt(blue_ball_Y.at(i) * blue_ball_Y.at(i) + blue_ball_X.at(i) * blue_ball_X.at(i));
		if(ball_dist < min_dist) {
			min_dist = ball_dist;
			min_dist_x = blue_ball_X.at(i);
		}
	}
	if(remaining_blue_ball_num == 0) {
		if(detected_green_ball_num > 0) {
			min_dist = std::sqrt(green_ball_Y.at(0) * green_ball_Y.at(0) + green_ball_X.at(0) * green_ball_X.at(0));
			min_dist_x = green_ball_X.at(0);
			if(min_dist_x < 0) {
				next_spin_direction = CCW;
			}
			else {
				next_spin_direction = CW;
			}
		}
		else {
			min_dist_x = 999.0f;
		}
	}
	if(min_dist == 100.0f && min_dist_x == 0.0f) {
		min_dist_x = 999.0f;
	}
	return command_to_align(min_dist, min_dist_x, 0.0f);
}

Wheel_commands collect_ball_update_lower() {
	//받아온 이미지에 파란공이 포착되지 않을 경우
	if(detected_lower_blue_ball_num == 0) {
		return Wheel_commands();
	}
	//파란공 3개를 모두 모았을 경우 remaining_blue_ball_num 변수를 0으로 만들어 골 지점으로 이동하는 알고리즘을 수행할 수 있도록 함.
	if(lower_blue_ball_Z.at(0) < 0.0020f && lower_blue_ball_Z.at(0) > -0.017f) {
		third_ball_count++;
		if(third_ball_count >= 100) {
			remaining_blue_ball_num = 0;
		}
	}
	//공을 보관하는 구조와 타겟이 되는 파란 공과의 x-방향의 정렬을 맞춰 파란 공이 성공적으로 구조 안으로 들어올 수 있도록 함.
	Wheel_commands result = command_to_align(std::sqrt(std::pow(lower_blue_ball_X.at(0), 2)),
											lower_blue_ball_X.at(0), 0.0f);
	return result;
}

Wheel_commands collect_ball_update_rear() {
	float min_dist = 100.0f;
	float min_dist_x = 0.0f;
	for(int i = 0; i < detected_rear_blue_ball_num; i++) {
		float ball_dist = std::sqrt(rear_blue_ball_Y.at(i) * rear_blue_ball_Y.at(i) + rear_blue_ball_X.at(i) * rear_blue_ball_X.at(i));
		if(ball_dist < min_dist) {
			min_dist = ball_dist;
			min_dist_x = blue_ball_X.at(i);
		}
	}

	if(min_dist == 100.0f && min_dist_x == 0.0f) {
		min_dist_x = 999.0f;
	}
	return command_to_align_rear(min_dist, min_dist_x / 2.0f, 0.0f);
}

float collect_ball_update_rear_dist() {
	float min_dist = 100.0f;
	float min_dist_x = 0.0f;
	for(int i = 0; i < detected_rear_blue_ball_num; i++) {
		float ball_dist = std::sqrt(rear_blue_ball_Y.at(i) * rear_blue_ball_Y.at(i) + rear_blue_ball_X.at(i) * rear_blue_ball_X.at(i));
		if(ball_dist < min_dist) {
			min_dist = ball_dist;
			min_dist_x = blue_ball_X.at(i);
		}
	}

	if(min_dist == 100.0f && min_dist_x == 0.0f) {
		min_dist_x = 999.0f;
	}
	return min_dist;
}

Wheel_commands collect_rear_ball_update() {
	float min_dist = 100.0f;
	float min_dist_x = 0.0f;
	for(int i = 0; i < detected_rear_blue_ball_num; i++) {
		float ball_dist = std::sqrt(rear_blue_ball_Y.at(i) * rear_blue_ball_Y.at(i) + rear_blue_ball_X.at(i) * rear_blue_ball_X.at(i));
		if(ball_dist < min_dist) {
			min_dist = ball_dist;
			min_dist_x = blue_ball_X.at(i);
		}
	}

	if(min_dist == 100.0f && min_dist_x == 0.0f) {
		min_dist_x = 999.0f;
	}
	return command_to_align_rear(min_dist, min_dist_x, 0.0f);
}

float collect_ball_update_rear_up_dist() {
	float min_dist = 100.0f;
	float min_dist_x = 0.0f;
	for(int i = 0; i < detected_rear_up_blue_ball_num; i++) {
		float ball_dist = std::sqrt(rear_up_blue_ball_Y.at(i) * rear_up_blue_ball_Y.at(i) + rear_up_blue_ball_X.at(i) * rear_up_blue_ball_X.at(i));
		if(ball_dist < min_dist) {
			min_dist = ball_dist;
			min_dist_x = rear_up_blue_ball_X.at(i);
		}
	}

	if(min_dist == 100.0f && min_dist_x == 0.0f) {
		min_dist_x = 999.0f;
	}
	return min_dist;
}

Wheel_commands collect_green_ball_update() {
	//Find a green ball with the maximum Y value @ lower cam
	float min_dist = 100.0f;
	float min_dist_x = 0.0f;
	for(int i = 0; i < detected_green_ball_num_lower; i++) {
		float ball_dist = std::sqrt(green_ball_Y_lower.at(i) * green_ball_Y_lower.at(i) + green_ball_X_lower.at(i) * green_ball_X_lower.at(i));
		if(ball_dist < min_dist) {
			min_dist = ball_dist;
			min_dist_x = green_ball_X_lower.at(i);
		}
	}
	if(min_dist == 100.0f && min_dist_x == 0.0f) {
		min_dist_x = 999.0f;
	}
	return command_to_align(min_dist, min_dist_x, 0.0f);
}

Wheel_commands collect_green_ball_update_rear() {
	//Find a green ball with the maximum Y value
	float min_dist = 100.0f;
	float min_dist_x = 0.0f;
	for(int i = 0; i < detected_green_ball_num_rear; i++) {
		float ball_dist = std::sqrt(green_ball_Y_rear.at(i) * green_ball_Y_rear.at(i) + green_ball_X_rear.at(i) * green_ball_X_rear.at(i));
		if(ball_dist < min_dist) {
			min_dist = ball_dist;
			min_dist_x = green_ball_X_rear.at(i);
		}
	}

	if(min_dist == 100.0f && min_dist_x == 0.0f) {
		min_dist_x = 999.0f;
	}
	return command_to_align_rear(min_dist, min_dist_x, 0.0f);
}


float collect_upper_green_ball_update_dist() {
	//I'm not sure it is an useful function...

	//Find a green ball with the minimum Y value @ upper cam
	float min_dist = 100.0f;
	float min_dist_x = 0.0f;
	for(int i = 0; i < detected_green_ball_num; i++) {
		float ball_dist = std::sqrt(green_ball_Y.at(i) * green_ball_Y.at(i) + green_ball_X.at(i) * green_ball_X.at(i));
		if(ball_dist < min_dist) {
			min_dist = ball_dist;
			min_dist_x = green_ball_X.at(i);
		}
	}
	if(min_dist == 100.0f && min_dist_x == 0.0f) {
		min_dist_x = 999.0f;
	}
	return min_dist;
}

Wheel_commands collect_rear_up_green_ball_update() {
	//Find a green ball with the minimum Y value
	float min_dist = 100.0f;
	float min_dist_x = 0.0f;
	for(int i = 0; i < detected_rear_up_green_ball_num; i++) {
		float ball_dist = std::sqrt(rear_up_green_ball_Y.at(i) * rear_up_green_ball_Y.at(i) + rear_up_green_ball_X.at(i) * rear_up_green_ball_X.at(i));
		if(ball_dist < min_dist) {
			min_dist = ball_dist;
			min_dist_x = rear_up_green_ball_X.at(i);
		}
	}

	if(min_dist == 100.0f && min_dist_x == 0.0f) {
		min_dist_x = 999.0f;
	}
	return command_to_align_rear(min_dist, min_dist_x, 0.0f);
}

float collect_rear_up_green_ball_update_dist() {
	//I'm not sure it is an useful function...

	//Find a green ball with the maximum Y value
	float min_dist = 100.0f;
	float min_dist_x = 0.0f;
	for(int i = 0; i < detected_rear_up_green_ball_num; i++) {
		float ball_dist = std::sqrt(rear_up_green_ball_Y.at(i) * rear_up_green_ball_Y.at(i) + rear_up_green_ball_X.at(i) * rear_up_green_ball_X.at(i));
		if(ball_dist < min_dist) {
			min_dist = ball_dist;
			min_dist_x = rear_up_green_ball_X.at(i);
		}
	}

	if(min_dist == 100.0f && min_dist_x == 0.0f) {
		min_dist_x = 999.0f;
	}
	return min_dist;
}

float collect_green_ball_update_rear_dist() {
	//I'm not sure it is an useful function...

	//Find a green ball with the maximum Y value
	float min_dist = 100.0f;
	float min_dist_x = 0.0f;
	for(int i = 0; i < detected_green_ball_num_rear; i++) {
		float ball_dist = std::sqrt(green_ball_Y_rear.at(i) * green_ball_Y_rear.at(i) + green_ball_X_rear.at(i) * green_ball_X_rear.at(i));
		if(ball_dist < min_dist) {
			min_dist = ball_dist;
			min_dist_x = green_ball_X_rear.at(i);
		}
	}

	if(min_dist == 100.0f && min_dist_x == 0.0f) {
		min_dist_x = 999.0f;
	}
	return min_dist;
}

void imu_Callback(const sensor_msgs::Imu::ConstPtr &imu) {
	angular_v_x = imu->angular_velocity.x;
	angular_v_y = imu->angular_velocity.y;
	angular_v_z = imu->angular_velocity.z;
	linear_a_x = imu->linear_acceleration.x;
	linear_a_y = imu->linear_acceleration.y;
	linear_a_z = imu->linear_acceleration.z;
}

float distance_to_line(Point point, Point start, Point end) {
	float dist_to_start = std::hypotf(point.x - start.x, point.y - start.y);
	float dist_to_end = std::hypotf(point.x - end.x, point.y - end.y);
	float line_length = std::hypotf(start.x - end.x, start.y - end.y);
	//Check if the point is out of the boundary
	if((dist_to_start*dist_to_start) < (dist_to_end*dist_to_end) - (line_length*line_length)) {
		return dist_to_start;
	}
	if((dist_to_end*dist_to_end) < (dist_to_start*dist_to_start) - (line_length*line_length)) {
		return dist_to_end;
	}
	return std::abs((end.y - start.y) * point.x - (end.x - start.x) * point.y + end.x * start.y - start.x * end.y)
			/ std::hypotf(end.y - start.y, end.x - start.x);
}

std::vector<Point> update_route(const float start_x, const float start_y, const float dest_x, const float dest_y) {
	Point start;
	start.x = start_x; start.y = start_y;
	Point dest;
	dest.x = dest_x; dest.y = dest_y;
	float length = std::hypotf(start_x - dest.x, start_y - dest.y); //로봇과 목표지점 사이의 직선 거리
	//Check if the start or dest is within the circle
	std::vector<Point> concerned_obstacles; //로봇이 움직임에 따라 맞닥뜨릴 장애물들의 좌표를 저장.
	for(Point obs: OBSTACLES) {
		if(std::hypotf(start_x - obs.x, start_y - obs.y) < 0.07f + 0.16) { //시작 지점 근처에 있는 장애물은 무시
			continue;
		}
		if(std::hypotf(dest_x - obs.x, dest_y - obs.y) < 0.07f + 0.16) { //목표 지점 근처에 있는 장애물은 무시
			continue;
		}
		concerned_obstacles.push_back(obs); //그 외 장애물들만 저장해놓은 변수
	}

	//Check collide, list the detour points
	std::vector<Point> detour_candidates;
	for(Point obs: concerned_obstacles) {
		if(distance_to_line(obs, start, dest) < 0.07f + 0.16) { //점과 직선 사이 최소 거리를 계산하여 그 범위 안에 장애물이 있는지 확인
			Point detour_point1;
			//시작점과 목표점을 연결한 직선과 90도가 되도록 왼쪽에 detour point 설정
			detour_point1.x = obs.x + (0.07f + 0.16 + 0.15f) * (dest.y - start_y) / length;
			detour_point1.y = obs.y - (0.07f + 0.16 + 0.15f) * (dest.x - start_x) / length;
			detour_candidates.push_back(detour_point1);
			Point detour_point2;
			//시작점과 목표점을 연결한 직선과 90도가 되도록 오른쪽에 detour point 설정
			detour_point2.x = obs.x - (0.07f + 0.16 + 0.15f) * (dest.y - start_y) / length;
			detour_point2.y = obs.y + (0.07f + 0.16 + 0.15f) * (dest.x - start_x) / length;
			detour_candidates.push_back(detour_point2);
		}
	}

	std::vector<Point> route;
	//경로 상에 obstacle이 없다면 dest 좌표 그대로를 목표로 반환한다.
	if(detour_candidates.empty()) {
		route.push_back(dest);
		return route;
	}
	else {
		//detour_point1과 detour_point2가 다른 장애물과 겹치는 경우 새로운 new_detour를 설정한다.
		std::vector<Point> new_detour_candidates;
		for(Point detour: detour_candidates) {
			Point new_detour;
			new_detour.x = -1.0f;
			for(Point obs: concerned_obstacles) {
				float dist = std::hypotf(detour.x - obs.x, detour.y - obs.y); //detour_point들과 obstacle간의 거리 측정
				if(dist < 0.07f + 0.16) { //detour_point가 obstacle의 danger zone에 있을 경우 detour_point를 새롭게 설정한다.
					Point detour_point1;
					detour_point1.x = obs.x + (0.07f + 0.16 + 0.15f) * (dest.y - start_y) / length;
					detour_point1.y = obs.y - (0.07f + 0.16 + 0.15f) * (dest.x - start_x) / length;
					Point detour_point2;
					detour_point2.x = obs.x - (0.07f + 0.16 + 0.15f) * (dest.y - start_y) / length;
					detour_point2.y = obs.y + (0.07f + 0.16 + 0.15f) * (dest.x - start_x) / length;
					//두 detour_point 중에서 obstacle의 중심으로부터 더 멀리 떨어진 포인트를 목표지점으로 설정한다.
					if(distance_to_line(detour_point1, start, dest) > distance_to_line(detour_point2, start, dest)) {
						new_detour = detour_point1;
					}
					else {
						new_detour = detour_point2;
					}
					break;
				}
			}
			//detour_point1과 detour_point2가 다른 장애물과 겹치지 않는 경우 그대로 detour_point를 가져간다.
			if(new_detour.x == -1.0f) {
				new_detour = detour;
			}
			new_detour_candidates.push_back(new_detour);
		}
		//Choose minimum distance
		float min_dist = 10000.0f;
		Point min_dist_detour;
		for(Point detour: new_detour_candidates) {
			//float dist = std::hypotf(start.x - detour.x, start.y - detour.y);
			float dist = distance_to_line(detour, start, dest);
			if(min_dist > dist) {
				min_dist = dist;
				min_dist_detour = detour;
			}
		}
		std::vector<Point> route_0 = {min_dist_detour};
		//std::vector<Point> route_1 = update_route(start_x, start_y, min_dist_detour.x, min_dist_detour.y);
		//std::vector<Point> route_2 = update_route(min_dist_detour.x, min_dist_detour.y, dest_x, dest_y);
		//route_1.insert(route_1.end(), route_2.begin(), route_2.end());
		//std::cout<<route_0.at(0).x << " " << route_0.at(0).y <<std::endl;
		return route_0;
	}
}

inline std::vector<Point> get_route(float dest_x, float dest_y) { //dest_x, dest_y는 로봇의 목표지점의 절대좌표
	return update_route(get_robot_end_x(), get_robot_end_y(), dest_x, dest_y);
	//get_robot_end_x()와 get_robot_end_y()는 로봇의 현재 위치를 나타내는 절대좌표
}

Point get_closest_blue_ball_position(const std::vector<Point> positions) {
	float min_dist = 100.0f;
	Point min_dist_position;
	min_dist_position.x = 0.0f;
	min_dist_position.y = 0.0f;
	for(int i = 0; i < (int)positions.size(); i++) {
		float ball_dist = std::hypotf(get_robot_end_x() - positions.at(i).x, get_robot_end_y() - positions.at(i).y);
		if(ball_dist < min_dist) {
			min_dist = ball_dist;
			min_dist_position = positions.at(i);
		}
	}
	return min_dist_position;
}

Wheel_commands move_to(Point dest) {
	Wheel_commands commands;
	float target_angle = RAD2DEG(std::atan2(dest.y - get_robot_end_y(), dest.x - get_robot_end_x()));
	float x = get_robot_deg() - target_angle;
	x = std::remainder(x, 360.0f);
	if(x > 210.0f) x -= 360.0f; //If x > 180 degree, turn right
	float distance = std::hypotf(dest.y - get_robot_end_y(), dest.x - get_robot_end_x());
	return motor_control(x / 90.0f, distance * saturate(std::cos(DEG2RAD(x)/2.0), 1.0f, 0.0f), PREVIOUS_ERROR, INTEGRAL_ERROR);
}

bool line_trace_over = false;
void line_trace_Callback(const std_msgs::Bool::ConstPtr& data) {
	line_trace_over = data -> data;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_integation");
	std::cout << "data_integration_node\n";
	ros::NodeHandle nh;
	//Line tracing 노드가 끝나고 data_integrate 노드로 넘어오기 위해 필요한 subscriber
	ros::Subscriber sub_line_tracing_over = nh.subscribe<std_msgs::Bool>("/line_trace_over", 2, line_trace_Callback);
	ros::Rate loop_rate(loop_frequency);
	while(!line_trace_over && ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Line tracing is over\n";

  //이 지점부터 Line tracing 종료 후 실행됨.

	//카메라 4개로부터 파란공의 위치 정보를 받아오는 subscriber 선언
	ros::Subscriber sub_blue_upper = nh.subscribe<core_msgs::ball_position>("/position_blue", 2, position_blue_upper_Callback);
	ros::Subscriber sub_blue_lower = nh.subscribe<core_msgs::ball_position_z>("/position_blue_lower", 2, position_blue_lower_Callback);
	ros::Subscriber sub_blue_rear = nh.subscribe<core_msgs::ball_position>("/position_blue_rear", 2, position_blue_rear_Callback);
	ros::Subscriber sub_rear_up_blue = nh.subscribe<core_msgs::ball_position>("/position_rear_up_blue", 2, position_rear_up_blue_Callback);

	//카메라 4개로부터 초록공의 위치 정보를 받아오는 subscriber 선언
	ros::Subscriber green_ball = nh.subscribe<core_msgs::ball_position>("/position_green", 2, green_ball_Callback);
	ros::Subscriber green_ball_lower = nh.subscribe<core_msgs::ball_position_z>("/position_green_lower", 2, green_ball_Callback_lower);
	ros::Subscriber green_ball_rear = nh.subscribe<core_msgs::ball_position>("/position_green_rear", 2, green_ball_Callback_rear);
	ros::Subscriber sub_rear_up_green_ball = nh.subscribe<core_msgs::ball_position>("/position_rear_up_green", 2, rear_up_green_ball_Callback);

	//카메라 4개로부터 빨간공의 위치 정보를 받아오는 subscriber 선언
	ros::Subscriber sub_red_upper = nh.subscribe<core_msgs::ball_position>("/position_red", 2, position_red_Callback);
	ros::Subscriber sub_red_rear = nh.subscribe<core_msgs::ball_position>("/position_red_rear", 2, position_red_rear_Callback);
	ros::Subscriber sub_red_lower = nh.subscribe<core_msgs::ball_position_z>("/position_red_lower", 2, position_red_lower_Callback);
	ros::Subscriber sub_rear_up_red = nh.subscribe<core_msgs::ball_position>("/position_rear_up_red", 2, position_rear_up_red_Callback);

  //Lidar와 Imu 센서 데이터를 받아 오기 위한 subscriber 선언
	ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("/imu_test", 2, imu_Callback);
	ros::Subscriber sub_lidar = nh.subscribe<sensor_msgs::LaserScan>("/scan", 2, lidar_Callback);
	robot_position_init(nh);

	//앞쪽의 양 바퀴를 제어하기 위한 publisher 선언
	pub_left_wheel= nh.advertise<std_msgs::Float64>("/cheetos/front_left_wheel_velocity_controller/command", 10);
	pub_right_wheel= nh.advertise<std_msgs::Float64>("/cheetos/front_right_wheel_velocity_controller/command", 10);

  //Data Show 노드에서 로봇의 목표 Path를 Visualize 해주기 위한 publisher. 실질적으로 알고리즘에 영향을 주진 않는다.
	ros::Publisher pub_route = nh.advertise<core_msgs::ball_position>("/route", 10);

  //이 지점부터 공 모으기 알고리즘을 수행.
	std::cout<<"Moving robot…"<<std::endl;
	bool found_all_blue_balls = false; //파란공을 2개 이상 발견했는 다 모았는지 여부를 판단하는 bool 변수
	std::cout<<"collecting balls"<<std::endl;

  //파란공 2개 이상을 발견할 때까지 제자리에서 회전한다.
	while (ros::ok() && !found_all_blue_balls){
		ros::spinOnce();
		loop_rate.sleep();
		rotate_to_detect(); //카메라 상에 보이는 파란공의 갯수를 지속적으로 업데이트 하면서 제자리 회전하는 함수
		found_all_blue_balls = (detected_blue_ball_num >= 2); //맨 처음 위치에서 파란공이 2개 이상 발견되면 회전을 멈추고 공 모으기 알고리즘을 수행.
		std::cout<<"blue balls:"<<detected_blue_ball_num<<std::endl;
	}

	std::cout<<"Found 3 balls"<<std::endl;
	while (ros::ok()) {
		ros::spinOnce();
		std::cout.precision(3);

    //파란공 3개를 다 모으고 골에 접근할 때 쓰이는 변수. 초록공이 발견 되고 그 거리가 0.6m보다 가깝다면 공을 밀어넣는 알고리즘을 수행한다.
		bool near_goal = (detected_green_ball_num > 0 && green_ball_Y.at(0) < 0.6f);
		//앞에 장애물이나 벽이 있다면 그것을 피하기 위해 필요한 변수.
		bool wall_in_front = (obstacle_front > 5);
		//로봇 뒤편에 장애물이나 벽이 있다면 회전할수 없으므로 그것을 방지하기 위해 앞으로 약간 이동할 때 필요한 변수.
		bool obstacle_in_back = (obstacle_side_back > 15);
		bool ball_stuck = (detected_lower_blue_ball_num > 0 && lower_blue_ball_Y.at(0) > 0.3f &&
							lower_blue_ball_Z.at(0) < 0.0f && std::abs(lower_blue_ball_X.at(0)) > 0.150f);
		bool catching_ball = (detected_lower_blue_ball_num > 0 && lower_blue_ball_Z.at(0) < -0.001);
		bool catching_red_ball = (detected_lower_red_ball_num > 0 && lower_red_ball_Y.at(0) < 0.3);
		bool catched_all = (remaining_blue_ball_num == 0);

		float goal_thresh_dist_lower = 0.04f;		// Need to find value using trial & error method
		float goal_thresh_angle = 0.01f;			// Need to find value using trial & error method
		float goal_thresh_dist_rear = 0.01f;		// Need to find value using trial & error method

    //파란공 3개를 모두 모았을 때 while 루프를 break하고 골을 찾아 이동하는 알고리즘을 수행한다.
		if(catched_all) {
			std::cout<<"Collected all balls\n";
			break;
		}
    /*
		로봇이 파란공을 줍기 위해 접근할 때 x-방향 정렬을 잘못 맞춰서 접근하면 공을 제대로 줍지 못하는 현상이 발생한다.
		이런 상황에서 벗어나기 위해 로봇은 다시 뒤로 약간 이동한다. ball_stuck 조건은 다음과 같이 정의된다.

		bool ball_stuck = (detected_lower_blue_ball_num > 0 && lower_blue_ball_Y.at(0) > 0.3f &&
							lower_blue_ball_Z.at(0) < 0.0f && std::abs(lower_blue_ball_X.at(0)) > 0.150f);

    파란공이 앞쪽 아래 카메라에 포착되어 공 모으기 알고리즘을 수행해야되는데 파란공이 로봇의 중앙으로부터 왼쪽이나 오른쪽으로
		0.150m 이상 떨어져 있으면 뒤로 이동하여 정렬을 다시 맞춘다.
		*/
		if(ball_stuck) {
			std::cout<<"Ball is stuck\n";
			Wheel_commands commands; //양쪽 바퀴에 publish 될 모터 인풋을 Wheel_commands structure로 정의하여 사용한다.
			commands.command_l.data = -2.0f;
			commands.command_r.data = -2.0f;
			pub_left_wheel.publish(commands.command_l);
			pub_right_wheel.publish(commands.command_r);
		}
		/*
    앞에 장애물이 있으면 피하기 위한  알고리즘을 수행한다.
		*/
		else if(check_obstacle_and_avoid()) {
			std::cout<<"Avoiding wall\n";
			PREVIOUS_ERROR = 0.0f;
			INTEGRAL_ERROR = 0.0f;
			pub_left_wheel.publish(left_wheel_command);
			pub_right_wheel.publish(right_wheel_command);
		}
		else if(catching_red_ball) {
			std::cout<<"Catching RED ball\n";
			left_wheel_command.data = -2.0f;
			right_wheel_command.data = -2.0f;
			pub_left_wheel.publish(left_wheel_command);
			pub_right_wheel.publish(right_wheel_command);
		}
		else if(catching_ball) {
			std::cout<<"Catching ball\n";
			auto commands = collect_ball_update_lower();
			pub_left_wheel.publish(commands.command_l);
			pub_right_wheel.publish(commands.command_r);
		}
		/*
		else if(check_red_ball_and_avoid()) {
			std::cout<<"Avoiding red ball\n";
			PREVIOUS_ERROR = 0.0f;
			INTEGRAL_ERROR = 0.0f;
			pub_left_wheel.publish(left_wheel_command);
			pub_right_wheel.publish(right_wheel_command);
		}
		*/
		else if (obstacle_back_surrounding > 0) {
			left_wheel_command.data = -4.0f;
			right_wheel_command.data = -4.0f;
			std::cout << "Avoiding: Go forward\n";
			pub_left_wheel.publish(left_wheel_command);
			pub_right_wheel.publish(right_wheel_command);
		}
		else {
			Wheel_commands commands; //양쪽 바퀴의 motor 인풋을 저장하기 위해 미리 정의된 Wheel_commands structure 사용

      //맵 상의 파란 공들 중 로봇과 가장 가까운 파란공을 타겟으로 정함.
			Point closest_blue_ball_position = get_closest_blue_ball_position(BLUE_BALL_POSITION);

			//로봇 앞쪽 위에 달린 카메라를 통해 받아온 이미지 상에 더 이상 파란공이 남아 있지 않은 경우 실행.
			if(closest_blue_ball_position.x == 0) {
				std::cout<<"Finding blue ball\n";
				collect_ball_update_lower(); //로봇 앞쪽 아래 카메라를 사용하여 파란공을 공 보관 구조에 넣기 위한 함수
				commands = command_to_align(0.0f, ARBITRARY_NAN, 0.0f); //타겟이 되는 파란 공과의 정렬을 맞추기 위한 함수
			}
			//로봇 앞쪽 위에 달린 카메라를 통해 받아온 이미지 상에 파란공이 발견될 경우 실행.
			else {
				core_msgs::ball_position msg_route;
				//get_route() 함수는 현재 로봇 위치에서 타겟이 되는 파란공까지 장애물과 빨간공을 피해 이동하는 최단 루트를 반환한다.
				std::vector<Point> route = get_route(closest_blue_ball_position.x, closest_blue_ball_position.y);
				msg_route.size = route.size();
				for(Point route_p: route) {
					msg_route.img_x.push_back(route_p.x);
					msg_route.img_y.push_back(route_p.y);
				}
				//장애물이 없을 경우
				//msg_route.size = 1; msg_route.img_x.push_back(closest_blue_ball_position.x); msg_route.img_y.push_back(closest_blue_ball_position.y);
				pub_route.publish(msg_route);
				commands = move_to(route.at(0));
			}

			pub_left_wheel.publish(commands.command_l);
			pub_right_wheel.publish(commands.command_r);
			//std::cout<<"L:"<<std::fixed<<commands.command_l.data<<", R:"<<std::fixed<<commands.command_r.data<<std::endl;
		}

		loop_rate.sleep();
	}
	while(ros::ok()) {
		ros::spinOnce();
		bool wall_in_front = (obstacle_front > 5);
		bool obstacle_in_back = (obstacle_side_back > 15);
		bool catching_ball = (detected_lower_blue_ball_num > 0 && lower_blue_ball_Z.at(0) < -0.01);
		bool catched_all = (remaining_blue_ball_num == 0);
		bool near_goal = (detected_green_ball_num > 0 && green_ball_Y.at(0) < 0.18f);
		if(!near_goal) {
			Wheel_commands commands;
			Point goal;
			goal.x = 8.0f;
			goal.y = 1.5f;
			commands = move_to(get_route(goal.x, goal.y).at(0));
			pub_left_wheel.publish(commands.command_l);
			pub_right_wheel.publish(commands.command_r);
		}
		else if(check_obstacle_and_avoid()) {
			std::cout<<"Avoiding wall\n";
			PREVIOUS_ERROR = 0.0f;
			INTEGRAL_ERROR = 0.0f;
			pub_left_wheel.publish(left_wheel_command);
			pub_right_wheel.publish(right_wheel_command);
		}
		else if(catching_ball) {
			std::cout<<"Catching ball\n";
			auto commands = collect_ball_update_lower();
			pub_left_wheel.publish(commands.command_l);
			pub_right_wheel.publish(commands.command_r);
		}
		else if(check_red_ball_and_avoid()) {
			std::cout<<"Avoiding red ball\n";
			PREVIOUS_ERROR = 0.0f;
			INTEGRAL_ERROR = 0.0f;
			pub_left_wheel.publish(left_wheel_command);
			pub_right_wheel.publish(right_wheel_command);
		}
		else if (obstacle_back_surrounding > 0) {
			left_wheel_command.data = -4.0f;
			right_wheel_command.data = -4.0f;
			std::cout << "Avoiding: Go forward\n";
			pub_left_wheel.publish(left_wheel_command);
			pub_right_wheel.publish(right_wheel_command);
		}
		//Put remaining green balls in
		if(near_goal) {
			break;
		}

		loop_rate.sleep();
	}

	while (ros::ok()) {
		ros::spinOnce();


		std::cout << "Ball Pushing Algorithm!" << std::endl;

		bool obstacle_in_back = (obstacle_side_back > 15);

		float goal_thresh_dist_upper_green = 0.95f;		// Value using trial & error method, ID = 1
		float goal_thresh_dist_rear_up_blue = 0.45f;		// Value using trial & error method, ID = 4
		float goal_thresh_dist_rear_blue = 0.14f;		// Value using trial & error method, ID = 5
		float goal_thresh_dist_rear_up_green = 0.14f;		// Value using trial & error method, ID = 7
		float goal_thresh_dist_rear_up_green_back = 0.7f;	// Value using trial & error method, ID = 8

		float goal_thresh_angle = 0.0f;		// Value using trial & error method, ID =

		int id_cnt = 1; // You can control what process you will start by this number in this code

		float dist_error = 0.05f;	// Value using trial & error method
		float angle_error= 0.05f;	// Value using trial & error method


		ros::Rate loop_rate(50);


		//Move backward to turn around, ID = 1
		float temp_min_dist=collect_upper_green_ball_update_dist();
		if(fabs(temp_min_dist - goal_thresh_dist_upper_green) <= dist_error) {
			if( id_cnt < 2) {
				id_cnt = 2;
			}
		}
		while(fabs(temp_min_dist - goal_thresh_dist_upper_green) > dist_error && id_cnt == 1) {
			ros::spinOnce();
			std::cout << "ID : " << id_cnt << std::endl;
			std::cout << "Difference1 : " << fabs(temp_min_dist - goal_thresh_dist_upper_green)  << std::endl;

			left_wheel_command.data = -2.0f;
			right_wheel_command.data = -2.0f;
			pub_left_wheel.publish(left_wheel_command);
			pub_right_wheel.publish(right_wheel_command);

			temp_min_dist=collect_upper_green_ball_update_dist();
			std::cout << "Difference2 : " << fabs(temp_min_dist - goal_thresh_dist_upper_green)  << std::endl;

			loop_rate.sleep();
		}
		if( id_cnt < 2) {
			id_cnt = 2;
		}


		//Turning 180 degree to use backside structure, ID = 2
		if(detected_rear_up_green_ball_num > 0) {
			if( id_cnt < 3) {
				id_cnt = 3;
			}
		}
		while(detected_rear_up_green_ball_num == 0 && id_cnt ==2) {
			ros::spinOnce();

			std::cout << "ID : " << id_cnt << std::endl;
			int spin_direction = -1; // 1 : turn left, -1: turn right

			if (obstacle_in_back) {
				if (lidar_closest_front_degree > 180.0f) {
					//Turn left. Because axis is locatded near front, back structure might cause problem.
					std::cout << "Turn right!" << std:: endl;
					spin_direction = -1;

					left_wheel_command.data = -2.0f;
					right_wheel_command.data = 3.0f;
				}
				else {
					spin_direction = 1;
					std::cout << "Turn left!" << std:: endl;

					left_wheel_command.data = 3.0f;
					right_wheel_command.data = -2.0f;
				}
			}
			else {
				if(spin_direction == -1) {
					std::cout << "Keep turn right!" << std:: endl;

					left_wheel_command.data = 2.0f;
					right_wheel_command.data = -3.0f;
				}
				else {
					std::cout << "Keep turn left!" << std:: endl;

					left_wheel_command.data = -2.0f;
					right_wheel_command.data = 3.0f;
				}
			}

			pub_left_wheel.publish(left_wheel_command);
			pub_right_wheel.publish(right_wheel_command);

			loop_rate.sleep();
		}
		if( id_cnt < 3) {
			id_cnt = 3;
		}

		// Align green ball in the middle line of cam sight, ID = 3
		float temp_angle = rear_up_green_ball_X.at(0);
		if(fabs(temp_angle-goal_thresh_angle) <= angle_error) {
			if( id_cnt < 4) {
				id_cnt = 4;
			}
		}
		while(fabs(temp_angle-goal_thresh_angle) > angle_error && id_cnt == 3) {
			ros::spinOnce();

			std::cout << "ID : " << id_cnt << std::endl;
			std::cout << "Current angle : " << temp_angle << std::endl;

			if(temp_angle < 0) {
				std::cout << "Turn left!" << std:: endl;

				left_wheel_command.data = -3.0f;
				right_wheel_command.data = 2.0f;
			}
			else {
				std::cout << "Turn right!" << std:: endl;

				left_wheel_command.data = 2.0f;
				right_wheel_command.data = -3.0f;
			}

			pub_left_wheel.publish(left_wheel_command);
			pub_right_wheel.publish(right_wheel_command);

			temp_angle = rear_up_green_ball_X.at(0);

			loop_rate.sleep();
		}
		if( id_cnt < 4) {
			id_cnt = 4;
		}


		// Push the balls & move backward repeat to check the whole balls are in the goal
		while(detected_rear_up_blue_ball_num > 0) {
			ros::spinOnce();

			std::cout << "Blue ball in goal area!" << std::endl;

			ros::Rate loop_rate(50);
			i = i*2.0/5.0;


			// Move to the nearest blue ball using rear up camera, ID = 4
			temp_min_dist=collect_rear_up_green_ball_update_dist();
			if(fabs(temp_min_dist - goal_thresh_dist_rear_up_blue) <= dist_error) {
				if( id_cnt < 5) {
					id_cnt = 5;
				}
			}
			while(fabs(temp_min_dist - goal_thresh_dist_rear_up_blue) > dist_error && id_cnt == 4) {
				ros::spinOnce();

				std::cout << "ID : " << id_cnt << std::endl;

				auto commands = collect_ball_update_rear();


				commands.command_l.data *= -1;
				commands.command_r.data *= -1;
				pub_left_wheel.publish(commands.command_l);
				pub_right_wheel.publish(commands.command_r);

				temp_min_dist=collect_rear_up_green_ball_update_dist();
				std::cout << "Difference : " << fabs(temp_min_dist - goal_thresh_dist_rear_up_blue)  << std::endl;

				loop_rate.sleep();
			}
			if( id_cnt < 5) {
				id_cnt = 5;
			}

			// Move to the nearest blue ball using rear camera, ID = 5
			temp_min_dist=collect_ball_update_rear_up_dist();
			if(fabs(temp_min_dist - goal_thresh_dist_rear_blue) <= dist_error) {
				if( id_cnt < 6) {
					id_cnt = 6;
				}
			}
			while(fabs(temp_min_dist - goal_thresh_dist_rear_blue) > dist_error && id_cnt == 5) {
				ros::spinOnce();

				std::cout << "ID : " << id_cnt << std::endl;

				auto commands = collect_rear_ball_update();

				pub_left_wheel.publish(commands.command_l);
				pub_right_wheel.publish(commands.command_r);

				temp_min_dist=collect_ball_update_rear_up_dist();
				std::cout << "Difference : " << fabs(temp_min_dist - goal_thresh_dist_rear_blue)  << std::endl;

				loop_rate.sleep();
			}
			if( id_cnt < 6) {
				id_cnt = 6;
			}


			// Align and Move to the green ball, goal hole location. ID = 6
			temp_angle = rear_up_green_ball_X.at(0);
			if(temp_angle < angle_error) {
				if( id_cnt < 7) {
					id_cnt = 7;
				}
			}
			temp_min_dist=rear_up_green_ball_Y.at(0);
			while(temp_angle >= angle_error && id_cnt == 6) {
				ros::spinOnce();

				std::cout << "ID : " << id_cnt << std::endl;
				std::cout << "Current angle : " << temp_angle << std::endl;

				auto commands = collect_rear_up_green_ball_update();

				pub_left_wheel.publish(commands.command_l);
				pub_right_wheel.publish(commands.command_r);

				std::cout << "Left Distance : " << temp_min_dist << std::endl;
				temp_angle = rear_up_green_ball_X.at(0);
				temp_min_dist=rear_up_green_ball_Y.at(0);

				loop_rate.sleep();
			}
			if( id_cnt < 7) {
				id_cnt = 7;
			}


			// Push the ball to the grean ball, hole. ID = 7
			temp_min_dist=rear_up_green_ball_Y.at(0);
			if(fabs(temp_min_dist - goal_thresh_dist_rear_up_green) <= dist_error) {
				if( id_cnt < 8) {
					id_cnt = 8;
				}
			}
			while(fabs(temp_min_dist - goal_thresh_dist_rear_up_green) > dist_error && id_cnt == 7) {
				ros::spinOnce();

				std::cout << "ID : " << id_cnt << std::endl;
				std::cout << "Distance : " << temp_min_dist  << std::endl;

				auto commands = collect_rear_up_green_ball_update();

				pub_left_wheel.publish(commands.command_l);
				pub_right_wheel.publish(commands.command_r);

				temp_min_dist=rear_up_green_ball_Y.at(0);
				std::cout << "Difference : " << fabs(temp_min_dist - goal_thresh_dist_rear_up_green)  << std::endl;

				loop_rate.sleep();
			}
			if( id_cnt < 8) {
				id_cnt = 8;
			}


			//Move backward. ID = 8
			temp_min_dist=rear_up_green_ball_Y.at(0);
			if(fabs(temp_min_dist - goal_thresh_dist_rear_up_green_back) <= dist_error) {
				if( id_cnt < 9) {
					id_cnt = 9;
				}
			}
			while(fabs(temp_min_dist - goal_thresh_dist_rear_up_green_back) > dist_error && id_cnt == 8) {
				ros::spinOnce();

				std::cout << "ID : " << id_cnt << std::endl;

				left_wheel_command.data = 2.0f;
				right_wheel_command.data = 2.0f;

				pub_left_wheel.publish(left_wheel_command);
				pub_right_wheel.publish(right_wheel_command);

				temp_min_dist=collect_rear_up_green_ball_update_dist();
				std::cout << "Difference : " << fabs(temp_min_dist - goal_thresh_dist_rear_up_green_back)  << std::endl;

				loop_rate.sleep();
			}
			if( id_cnt < 9) {
				id_cnt = 9;
			}
			// Reset id_cnt to repeat this process.
			if(id_cnt > 8) {
				id_cnt = 4;
			}


			loop_rate.sleep();
		}
		break;

		loop_rate.sleep();
	}

	while(ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	//
  return 0;
}
