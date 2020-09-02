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

float now_y = 0;
float pre_y = 0;
float start_threshold = 0.4;
float end_threshold = -0.3;
int slope = 0;
int count_y = 0;


void imu_Callback(const sensor_msgs::Imu::ConstPtr &imu) {
	now_y = imu->angular_velocity.y;
	if(slope == 0){
		if(pre_y>start_threshold && now_y>start_threshold) count_y++;
		else if(pre_y<start_threshold) count_y = 0;
		if(count_y > 20){
			slope = 1;
			count_y = 0;
			cout<<"slope start!!"<<endl;
		}
	}
	if(slope == 1){
		if(pre_y<end_threshold && now_y<end_threshold) count_y++;
		else if(pre_y>end_threshold) count_y = 0;
		if(count_y > 10){
			slope = 2;
			cout<<"slope end!!"<<endl;
		}
	}
	pre_y = now_y;
	//cout<<now_y<< endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_data");
	ros::NodeHandle nh;
	ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("/imu", 1000, imu_Callback);

	ros::Rate loop_rate(30);
	string filePath = "/home/cheetos/map_yangular_ver4.txt";
	ofstream txtFile(filePath);
	while(ros::ok()){
			ros::spinOnce();
			txtFile <<"y,"<<now_y<< endl;
			loop_rate.sleep();
	}
	txtFile.close();
  return 0;
}
