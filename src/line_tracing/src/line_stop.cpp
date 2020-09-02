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


#include <ros/ros.h>
#include <ros/package.h>
//#include "core_msgs/line_info.h"

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"


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

//**Variables for saving images function
char buf[256];
int count_1 = 0;

// Setting Mat variables for images.
Mat buffer;
Mat image_1;
Mat image_2;
Mat hsv_image;
Mat hsv_frame_black;
Mat img_canny_black;

float Kp = 0.02;
float Ki = 0.0001;
float Kd = 0.01;
float feedback = 0.0;
int now_error = 0;
int error_sum = 0;
int error_diff = 0;
int prev_error = 0;
float maxvel = 15.0;
float minvel = -5.0;
float vel = 5.0;
float left_wheel = 0.0;
float right_wheel = 0.0;

void check_vel() {
	if(left_wheel > maxvel){
		left_wheel = maxvel;
		cout << "lef too fast" <<endl;}
	else if(left_wheel < minvel) left_wheel = minvel;
	if(right_wheel > maxvel) right_wheel = maxvel;
	else if(right_wheel < minvel) right_wheel = minvel;
	cout << "check vel" <<endl;
}

void motor_control() {
		std_msgs::Float64 left_wheel_command;
		std_msgs::Float64 right_wheel_command;
		error_sum = now_error + error_sum;
		error_diff = now_error - prev_error;
		prev_error = now_error;
		feedback = Kp*now_error + Ki*error_sum + Kd*error_diff;

		left_wheel = vel - feedback;
		right_wheel = vel + feedback;
		check_vel();
		cout << left_wheel << "    "  << right_wheel << "\n";

		left_wheel_command.data = 0;
		right_wheel_command.data = 0;


		pub_left_wheel.publish(left_wheel_command);
		pub_right_wheel.publish(right_wheel_command);
}

void image_detect()
{
  Mat intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
  Mat distCoeffs = Mat(1, 5, CV_32F, distortion_data);

  // Declare another Mat variable to keep the image.
  Mat frame;
  frame = buffer;

  Mat calibrated_frame;
  // Undistort frame images and save to calibrated frame.
  undistort(frame, calibrated_frame, intrinsic, distCoeffs);

  //**truncating image from camera
  image_2=calibrated_frame.clone();
  Rect rect(480, 540, 960, 540);
  image_1=image_2(rect);
  imshow("Frame1",image_1);

  //**line offset code
  cvtColor(image_1, hsv_image, COLOR_BGR2HSV);
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
  for (int i = 60; i<540; i+=60)
  {
      points.clear();
      for (int j=0; j<320; j++)
      {
          Vec3b intensity = img_canny_black.at<Vec3b>(i,j);
          for (int z=0; z<3 ; z++)
          {
              if (intensity.val[z] == 255)
              {
                  points.push_back(3*j+z+1);
              }
          }
      }
      int n = points.size();
      if (n != 0)
      {
          float average = accumulate( points.begin(), points.end(), 0.0)/points.size();
          float offsetofpoint = 480 - average;
          average_distances.push_back(i);
          average_distances.push_back(offsetofpoint);
      }
  }
  int num = average_distances.size();
	if(num != 0) now_error = average_distances[num-1];
	else now_error = 0;
	/*
  for (int y=0; y<num; y++)
  {
      //cout << average_distances[y] << "\n";
			now_error = now_error + average_distances[y];
  }
	if(num != 0) now_error = now_error / num;*/

	cout << now_error << "\n";
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if(msg->height==480&&buffer.size().width==320) {  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
    std::cout<<"resized"<<std::endl;
    cv::resize(buffer,buffer,cv::Size(640,480));
  }
  try {buffer = cv_bridge::toCvShare(msg, "bgr8")->image;} //transfer the image data into buffer
  catch (cv_bridge::Exception& e) {ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());}
  image_detect();
	motor_control();
  waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_stop");
	  ros::NodeHandle n;
	  image_transport::ImageTransport it(n); //create image transport and connect it to node hnalder
	  image_transport::Subscriber sub = it.subscribe("/lower_camera/rgb/image_raw", 1, imageCallback);
		pub_left_wheel= n.advertise<std_msgs::Float64>("/cheetos/front_left_wheel_velocity_controller/command", 10);
		pub_right_wheel= n.advertise<std_msgs::Float64>("/cheetos/front_right_wheel_velocity_controller/command", 10);

    //ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
		ros::Rate loop_rate(30);
	  while(ros::ok()){
	      ros::spinOnce();
	      loop_rate.sleep();
	  }
    // Rotate until the robot detects all the blue balls in the map.

    return 0;
}
