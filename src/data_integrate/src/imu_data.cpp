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
#include "string.h"

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

using namespace std;

float angular_v_x;
float angular_v_y;
float angular_v_z;
float linear_a_x;
float linear_a_y;
float linear_a_z;
float z_vel = 0;
float z_pos = 0;
float y_sum = 0;
void imu_Callback(const sensor_msgs::Imu::ConstPtr &imu) {
	angular_v_x = imu->angular_velocity.x;
	angular_v_y = imu->angular_velocity.y;
	angular_v_z = imu->angular_velocity.z;
	linear_a_x = imu->linear_acceleration.x;
	linear_a_y = imu->linear_acceleration.y;
	linear_a_z = imu->linear_acceleration.z;
	cout<<linear_a_z<< endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_data");
	ros::NodeHandle nh;
	ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("/imu", 1000, imu_Callback);

	ros::Rate loop_rate(30);
	string filePath = "/home/cheetos/slope_ver2.txt";
	ofstream txtFile(filePath);
	while(ros::ok()){
			ros::spinOnce();
			txtFile <<"x,"<<linear_a_x<<",y,"<<linear_a_y<<",z,"<<linear_a_z<< endl;
			loop_rate.sleep();
	}
	txtFile.close();
  return 0;
}
