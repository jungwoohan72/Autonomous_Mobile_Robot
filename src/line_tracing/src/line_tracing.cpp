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
#include "string.h"
#include <ros/ros.h>
#include <ros/package.h>
#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"
#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <numeric>

using namespace std;
using namespace cv;

// Initialization of variable for camera calibration paramters.
float intrinsic_data[9] = {1206.8897719532354, 0.0, 960.5, 0.0, 1206.8897719532354, 540.5, 0.0, 0.0, 1.0};
float distortion_data[5] = {0, 0, 0, 0, 0};

//Setting Publisher
ros::Publisher pub_left_wheel;
ros::Publisher pub_right_wheel;
ros::Publisher pub_left_sus;
ros::Publisher pub_right_sus;
ros::Publisher pub_rear_sus;

ros::Publisher pub_line_trace_state;

std_msgs::Bool msg_line_tracing_over;

//**Variables for saving images function
char buf[256];
int count_1 = 0;

// Setting Mat variables for images.
Mat buffer;
Mat image_2;
Mat hsv_image;
Mat hsv_frame_black;
Mat img_canny_black;

float Kp = 0.003;
float Ki = 0.0005;
float Kd = 0.00;
double feedback = 0.0;
int now_error = 0;
int center_error = 0;
int lower_error = 0;
int up_error = 0;
int prev_error = 0;
int error_sum = 0;
int error_diff = 0;

double vel = 0.0;
double left_wheel = 0.0;
double right_wheel = 0.0;

int count_white = 0;
int num = 0;
int lower_num = 0;

float now_y = 0;
float pre_y = 0;
float start_threshold = -0.25;
float end_threshold = 0.5;
int slope = 0;
int count_y = 0;
int slope_entrance = -1;

int bump = 0;
int count_bump = 0;
int bump_wait = 0;
int lower_k = 2;
int lower_bump = 0;
int bump_end = 0;

void imu_Callback(const sensor_msgs::Imu::ConstPtr &imu) {
	now_y = imu->angular_velocity.y;
	if(slope == 0){
		if(bump == 2 && slope_entrance==4 && now_y<start_threshold) count_y++;
		if(count_y > 13){
			slope = 1;
			count_y = 0;
			Kp = 0.01;
			Ki = 0;
			Kd = 0;
			cout<<"slope start!!"<<endl;
		}
	}
	if(slope == 1){
		if(pre_y>end_threshold && now_y>end_threshold) count_y++;
		else if(pre_y<end_threshold) count_y = 0;
		if(count_y > 10){
			slope = 2;
			bump_wait = 0;
			Kp=0.013;
			Kd=0.03;
			cout<<"slope end!!"<<endl;
		}
	}
	pre_y = now_y;
}

void check_vel(double maxvel, double minvel) {
	if(left_wheel > maxvel) left_wheel = maxvel;
	else if(left_wheel < minvel) left_wheel = minvel;
	if(right_wheel > maxvel) right_wheel = maxvel;
	else if(right_wheel < minvel) right_wheel = minvel;
}

void motor_control() {
		std_msgs::Float64 left_wheel_command;
		std_msgs::Float64 right_wheel_command;
		std_msgs::Float64 left_sus_command;
		std_msgs::Float64 right_sus_command;
		std_msgs::Float64 rear_sus_command;

		error_sum = now_error + error_sum;
		error_diff = now_error - prev_error;
		prev_error = now_error;
		feedback = Kp*now_error + Ki*error_sum + Kd*error_diff;

		if(bump==0) vel = 6;
		else if(bump==1) vel = 10.0;
		else if(bump==2){
			if(slope==0) vel = 2.0;
			else if(slope==1) vel = 7.0;
			else vel = 3.0;
		}
		else if(bump==3) vel = 5.0;
		else vel = 2.0;

		left_wheel = vel + feedback;
		right_wheel = vel - feedback;

		if(bump==1) check_vel(10.0, 10.0);

		if(bump==2 && slope==0){
			if(slope_entrance<0 && lower_k==2){
				left_wheel = 10.0;
				right_wheel = 10.0;
				cout<<"lower_k is "<<lower_k<<endl;
			}
			else if(slope_entrance < 1 && (num<34 || lower_num>2 || center_error > -3)){
				left_wheel = 3;
				right_wheel = -3;
				slope_entrance = 0;
			}
			else if(slope_entrance < 2 && (num>0 || lower_num<34)){
				slope_entrance = 1;
				left_wheel = 4;
				right_wheel = 4;
				bump_wait = 0;
			}
			else if(slope_entrance < 3 && lower_error < 55){
					left_wheel = 3;
					right_wheel = 3;
					slope_entrance = 2;
					bump_wait = 0;
			}
			else if(slope_entrance < 4 && bump_wait < 2){
					left_wheel = 4;
					right_wheel = 4;
					slope_entrance = 3;
					bump_wait++;
			}
			else if(slope_entrance < 4 && (num<34 || lower_num>0 || center_error > -3 || up_error > -2)){
				left_wheel = 3;
				right_wheel = -3;
				slope_entrance = 3;
			}
			else{
				left_wheel = 6;
				right_wheel = 3;
				slope_entrance = 4;
				bump_wait = 0;
			}
		}

		if(slope==1){
			if(bump_wait < 3 && slope_entrance == 0){
				left_wheel = 6;
				right_wheel = 2;
				slope_entrance = 0;
				bump_wait++;
			}
			else{
				check_vel(10,3);
				bump_wait = 0;
				slope_entrance = 1;
			}
		}

		if(bump==2 && slope==2){
			if(slope_entrance == 1 && num>0){
				left_wheel = right_wheel = 6;
			}
			else if(slope_entrance < 3 && num<34){
				left_wheel = -3;
				right_wheel = 3;
				error_sum=0;
				error_diff=0;
				prev_error=0;
				slope_entrance = 2;
				bump_wait = 0;
			}
			else if(bump_end < 10){
				check_vel(8.7,-3);
				slope_entrance = 3;
				if(num == 0) bump_end++;
				else bump_end = 0;
				if(left_wheel > 4) left_wheel = 4;
			}
			else bump = 3;
		}

		if(bump==3){
			if(slope_entrance < 5 && (center_error < -3 || up_error < -300)){
				left_wheel = -5;
				right_wheel = 6;
				lower_k = 2;
				lower_bump = 0;
			}
			else{
				check_vel(6,4);
				slope_entrance = 5;
				cout << lower_k << endl;
			}
		}
		if(bump==4) {
			cout << lower_k << endl;
			if(lower_k==2){
				left_wheel = 2;
				right_wheel = 2;
			}
			else if(slope_entrance < 6 && (lower_num>2 || now_error > -3)){
				left_wheel = 4;
				right_wheel = -3;
				cout<<"center_error is "<<center_error<<"    lower_num is "<<lower_num<<endl;
			}
			else{
				slope_entrance = 6;
				left_wheel = 2;
				right_wheel = 2;
			}
		}

		left_wheel_command.data = -left_wheel;
		right_wheel_command.data = -right_wheel;

		cout << right_wheel << "    "  << left_wheel << "\n";

		left_sus_command.data = 0;
		right_sus_command.data = 0;
		rear_sus_command.data = 0;
		if(count_white > 10){
			left_wheel_command.data = 0;
			right_wheel_command.data = 0;
			msg_line_tracing_over.data = true;
			cout << "-------line tracing over--------" <<endl;
		}
		if(!msg_line_tracing_over.data) {
			pub_left_wheel.publish(left_wheel_command);
			pub_right_wheel.publish(right_wheel_command);
		}
		pub_left_sus.publish(left_sus_command);
		pub_right_sus.publish(right_sus_command);
		pub_rear_sus.publish(rear_sus_command);
		pub_line_trace_state.publish(msg_line_tracing_over);
}

void image_detect()
{
	Mat intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
  Mat distCoeffs = Mat(1, 5, CV_32F, distortion_data);
  Mat frame;
  frame = buffer;
  Mat calibrated_frame;
  undistort(frame, calibrated_frame, intrinsic, distCoeffs);
  image_2=calibrated_frame.clone();
  cvtColor(image_2, hsv_image, COLOR_BGR2HSV);
  vector<float> average_distances;
  int low_h_b = 0;
  int high_h_b = 180;
  int low_s_b = 0;
  int high_s_b = 255;
  int low_v_b = 0;
  int high_v_b = 40;
  inRange(hsv_image, Scalar(low_h_b,low_s_b,low_v_b), Scalar(high_h_b,high_s_b,high_v_b),hsv_frame_black);
  int lowThreshold=100;
  int ratio=3;
  int kernel_size=3;
  Canny(hsv_frame_black, img_canny_black, lowThreshold, lowThreshold*ratio, kernel_size);
  vector<float>points;
  for (int i = 60; i<1080; i+=60){
    points.clear();
    for (int j=0; j<640; j++){
        Vec3b intensity = img_canny_black.at<Vec3b>(i,j);
        for (int z=0; z<3 ; z++){
            if (intensity.val[z] == 255) points.push_back(3*j+z+1);
        }
    }
    int n = points.size();
    if (n != 0){
    	float average = accumulate( points.begin(), points.end(), 0.0)/points.size();
      float offsetofpoint = 960 - average;
      average_distances.push_back(i);
      average_distances.push_back(offsetofpoint);
  	}
  }
  num = average_distances.size();
	if(num != 0){
		now_error = average_distances[num-1];
		center_error = average_distances[num/2];
		up_error = average_distances[1];
	}
	else if(bump == 4 && slope_entrance == 6){
		now_error = 0;
		error_sum=0;
		error_diff=0;
		prev_error=0;
		count_white++;
	}

	vector<float> numberofblacks;
  vector<float>pointstwo;
  int p = 0;
  for (int i = 0; i<1080; i++){
      pointstwo.clear();
      for (int j=0; j<640; j++){
        Vec3b intensity = img_canny_black.at<Vec3b>(i,j);
        for (int z=0; z<3 ; z++){
            if (intensity.val[z] == 255) pointstwo.push_back(3*j+z+1);
        }
      }
      int n = pointstwo.size();
      if (n != 0)  p = p+1;
      if (n == 0){
        if (p != 0) numberofblacks.push_back(p);
        p = 0;
      }
      if (i == 1079){
        if (p != 0) numberofblacks.push_back(p);
      }
  }
	int k = numberofblacks.size();

	if(bump==0 && k==2) bump=1;
	if(bump==1){
		if(k==2) count_bump=0;
		else count_bump++;
		if(count_bump>6){
			bump = 2;
			count_bump=0;
		}
	}
	if(bump==2){
		if(slope==2 && k>=2) count_bump++;
		if(count_bump>5){
			bump=3;
		}
	}
	if(bump==3){
		if(k>=2) count_bump=0;
		else count_bump++;
		if(slope_entrance==5 && count_bump>4) bump = 4;
	}
  cout << "bump is " << bump << "     k is " << k << "\n";
}

void lower_image_detect()
{
	Mat intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
  Mat distCoeffs = Mat(1, 5, CV_32F, distortion_data);
  Mat frame;
  frame = buffer;
  Mat calibrated_frame;
  undistort(frame, calibrated_frame, intrinsic, distCoeffs);
  image_2=calibrated_frame.clone();
  cvtColor(image_2, hsv_image, COLOR_BGR2HSV);
  vector<float> average_distances;
  int low_h_b = 0;
  int high_h_b = 180;
  int low_s_b = 0;
  int high_s_b = 255;
  int low_v_b = 0;
  int high_v_b = 40;
  inRange(hsv_image, Scalar(low_h_b,low_s_b,low_v_b), Scalar(high_h_b,high_s_b,high_v_b),hsv_frame_black);
  int lowThreshold=100;
  int ratio=3;
  int kernel_size=3;
  Canny(hsv_frame_black, img_canny_black, lowThreshold, lowThreshold*ratio, kernel_size);
  vector<float>points;
  for (int i = 60; i<1080; i+=60){
    points.clear();
    for (int j=0; j<640; j++){
        Vec3b intensity = img_canny_black.at<Vec3b>(i,j);
        for (int z=0; z<3 ; z++){
            if (intensity.val[z] == 255) points.push_back(3*j+z+1);
        }
    }
    int n = points.size();
    if (n != 0){
    	float average = accumulate( points.begin(), points.end(), 0.0)/points.size();
      float offsetofpoint = 960 - average;
      average_distances.push_back(i);
      average_distances.push_back(offsetofpoint);
  	}
  }
  lower_num = average_distances.size();
	if(lower_num != 0) lower_error = average_distances[num/2];

	vector<float> numberofblacks;
  vector<float>pointstwo;
  int p = 0;
  for (int i = 0; i<1080; i++){
      pointstwo.clear();
      for (int j=0; j<640; j++){
        Vec3b intensity = img_canny_black.at<Vec3b>(i,j);
        for (int z=0; z<3 ; z++){
            if (intensity.val[z] == 255) pointstwo.push_back(3*j+z+1);
        }
      }
      int n = pointstwo.size();
      if (n != 0)  p = p+1;
      if (n == 0){
        if (p != 0) numberofblacks.push_back(p);
        p = 0;
      }
      if (i == 1079){
        if (p != 0) numberofblacks.push_back(p);
      }
  }
	int k = numberofblacks.size();
	cout<<"lower k is what ?? " << k<<endl;
	if(bump==2 && k==1) lower_bump++;
	else if(bump==2) lower_bump = 0;
	if(lower_bump > 1) lower_k = 1;

	if(bump==4 && k==1) lower_bump++;
	else if(bump==4) lower_bump = 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if(msg->height==480&&buffer.size().width==320) {
    std::cout<<"resized"<<std::endl;
    cv::resize(buffer,buffer,cv::Size(640,480));
  }
  try {buffer = cv_bridge::toCvShare(msg, "bgr8")->image;}
  catch (cv_bridge::Exception& e) {ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());}
  image_detect();
	motor_control();
}

void lower_imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if(msg->height==480&&buffer.size().width==320) {
    std::cout<<"resized"<<std::endl;
    cv::resize(buffer,buffer,cv::Size(640,480));
  }
  try {buffer = cv_bridge::toCvShare(msg, "bgr8")->image;}
  catch (cv_bridge::Exception& e) {ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());}
  lower_image_detect();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "line_tracing_node");
	  ros::NodeHandle n;
	  image_transport::ImageTransport it(n); //create image transport and connect it to node hnalder
		ros::Subscriber sub_imu = n.subscribe<sensor_msgs::Imu>("/imu", 1000, imu_Callback);
		image_transport::Subscriber lower_sub = it.subscribe("/lower_camera/rgb/image_raw", 1, lower_imageCallback);
	  image_transport::Subscriber sub = it.subscribe("/rear_camera/rgb/image_raw", 1, imageCallback);
		pub_left_wheel= n.advertise<std_msgs::Float64>("/cheetos/front_left_wheel_velocity_controller/command", 10);
		pub_right_wheel= n.advertise<std_msgs::Float64>("/cheetos/front_right_wheel_velocity_controller/command", 10);
		pub_left_sus= n.advertise<std_msgs::Float64>("/cheetos/suspension_front_left/command", 10);
		pub_right_sus= n.advertise<std_msgs::Float64>("/cheetos/suspension_front_right/command", 10);
		pub_rear_sus= n.advertise<std_msgs::Float64>("/cheetos/suspension_rear/command", 10);
		pub_line_trace_state = n.advertise<std_msgs::Bool>("/line_trace_over", 10);

		msg_line_tracing_over.data = false;

		ros::Rate loop_rate(200);
	  while(ros::ok()){
	      ros::spinOnce();
	      loop_rate.sleep();
	  }
    return 0;
}
