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

//#include "core_msgs/markermsg.h"

using namespace std;
using namespace cv;

// Initialization of variable for camera calibration paramters.

float intrinsic_data[9] = {1206.8897719532354, 0.0, 960.5, 0.0, 1206.8897719532354, 540.5, 0.0, 0.0, 1.0};
float distortion_data[5] = {0, 0, 0, 0, 0};

//Setting Publisher
//ros::Publisher pub;

//**Variables for saving images function
char buf[256];
int count_1 = 0;

// Setting Mat variables for images.
Mat buffer;
Mat image_1;
Mat image_2;

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

  //**for saving images
  sprintf(buf, "/home/lsk/imagestore/img_%06d.jpg", count_1);
  imwrite(buf, image_1);
  count_1++;
  if (count_1==999999)
  {
	count_1 = 0;
  }

  //**for publishing images testing code below(compressed)

  //cv_bridge::CvImage img_bridges;
  //sensor_msgs::CompressedImage msgs;
  //std_msgs::Header header;
  //header.stamp = ros::Time::now();
  //img_bridges = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image_1);
  //img_bridges.toCompressedImageMsg(msgs);
  //pub.publish(msgs)
  //imshow("Frame1",image_1);

  //**for publishing images testing code below(uncompressed)

  //cv_bridge::CvImage img_bridge;
  //sensor_msgs::Image msg;
  //std_msgs::Header header;
  //header.stamp = ros::Time::now();
  //img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image_1);
  //img_bridge.toImageMsg(msg);
  //pub.publish(msg)
  //imshow("Frame1",image_1);

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
  //pub = n.advertise<sensor_msgs::CompressedImage>("camera_img", 100);
  //pub = n.advertise<sensor_msgs::Image>("camera_img", 100);
  ros::Rate loop_rate(30);
  while(ros::ok()){
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
