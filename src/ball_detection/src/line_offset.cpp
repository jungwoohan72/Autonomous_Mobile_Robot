#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <string>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <stdio.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <numeric>




using namespace std;
using namespace cv;

// Initialization of variable for camera calibration paramters.

float intrinsic_data[9] = {1206.8897719532354, 0.0, 960.5, 0.0, 1206.8897719532354, 540.5, 0.0, 0.0, 1.0};
float distortion_data[5] = {0, 0, 0, 0, 0};

//Setting Publisher
ros::Publisher pub;

//**Variables for saving images function
char buf[256];
int count_1 = 0;

// Setting Mat variables for images.
Mat buffer;
//Mat image_1;
Mat image_2;
Mat hsv_image;
Mat hsv_frame_black;
Mat img_canny_black;

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
  //Rect rect(480, 540, 960, 540);
  //image_1=image_2(rect);
  imshow("Frame1",image_2);

  //**line offset code
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
  //imshow("Frame2",img_canny_black);
  vector<float>points;
  for (int i = 60; i<1080; i+=60)
  {
      points.clear();
      //cout << "newline" << "\n";
      for (int j=0; j<640; j++)
      {
          Vec3b intensity = img_canny_black.at<Vec3b>(i,j);
          for (int z=0; z<3 ; z++)
          {
              if (intensity.val[z] == 255)
              {
                  //img_canny_black.at<Vec3b>(i,j)[z] = 0;
                  //cout << 3*j+z+1 << "\n";
                  points.push_back(3*j+z+1);
              }
          }
      }
      int n = points.size();
      if (n != 0)
      {
          //int sum = 0;
          //for (int k=0; k<n ; k++)
          //{
          //    sum+= points[k]+sum;
          //}
          //int average = sum/n;
          float average = accumulate( points.begin(), points.end(), 0.0)/points.size();
          float offsetofpoint = 960 - average;
          average_distances.push_back(i);
          average_distances.push_back(offsetofpoint);
      }
  }
  int num = average_distances.size();
  for (int y=0; y<num; y++)
  {
      //cout << average_distances[y] << "\n";
  }
  //publishing the data
  core_msgs::ball_position msgs;
  for (int g=0; g<num; g+=2)
  {
      msgs.img_x.push_back(average_distances[g]);
      msgs.img_y.push_back(average_distances[g+1]);
  }
  pub.publish(msgs);

  //**to distinguish if dotted or straight line
  cout << "black line portions" << "\n" ;
  vector<float> numberofblacks;
  vector<float>pointstwo;
  int p = 0;
  for (int i = 0; i<1080; i++)
  {
      pointstwo.clear();
      for (int j=0; j<640; j++)
      {
          Vec3b intensity = img_canny_black.at<Vec3b>(i,j);
          for (int z=0; z<3 ; z++)
          {
              if (intensity.val[z] == 255)
              {
                  pointstwo.push_back(3*j+z+1);
              }
          }
      }
      int n = pointstwo.size();
      if (n != 0)                //if black (in horizontal line of pixels)
      {
          p = p+1;
      }
      if (n == 0)                //if white (in horizontal line of pixels)
      {
          if (p != 0)
          {
              numberofblacks.push_back(p);
          }
          p = 0;
      }
      if (i == 1079)
      {
          if (p != 0)
          {
              numberofblacks.push_back(p);
          }
      }
  }
  cout << numberofblacks.size() << "\n";
  msgs.size = numberofblacks.size();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if(msg->height==480&&buffer.size().width==320) {  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
    std::cout<<"resized"<<std::endl;
    cv::resize(buffer,buffer,cv::Size(640,480));
  }
  try {buffer = cv_bridge::toCvShare(msg, "bgr8")->image;} //transfer the image data into buffer
  catch (cv_bridge::Exception& e) {ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());}
  image_detect();
  waitKey(3);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n); //create image transport and connect it to node hnalder
  image_transport::Subscriber sub = it.subscribe("/lower_camera/rgb/image_raw", 1, imageCallback);
  pub = n.advertise<core_msgs::ball_position>("/offsetvalues",1);
  ros::Rate loop_rate(30);
  while(ros::ok()){
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
