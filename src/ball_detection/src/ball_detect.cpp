#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
#include "core_msgs/ball_position_z.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include "std_msgs/Bool.h"
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

#define PI 3.14159265

using namespace std;
using namespace cv;

// Setting Thresholds for red and blue part of image.
// Changable to fit your enviroment. If you want to use bgr, it should be different.

// void on_low_h_thresh_trackbar_red_1(int, void *);
// void on_high_h_thresh_trackbar_red_1(int, void *);
// void on_low_h2_thresh_trackbar_red_1(int, void *);
// void on_high_h2_thresh_trackbar_red_1(int, void *);
// void on_low_s_thresh_trackbar_red_1(int, void *);
// void on_high_s_thresh_trackbar_red_1(int, void *);
// void on_low_v_thresh_trackbar_red_1(int, void *);
// void on_high_v_thresh_trackbar_red_1(int, void *);

int low_h_g=36, low_s_g=25, low_v_g=25;
int high_h_g=70, high_s_g=255, high_v_g=255;

int low_h2_r=169, high_h2_r=180;
int low_h_r=0, low_s_r=50, low_v_r=20;
int high_h_r=8, high_s_r=255, high_v_r=255;
int low_h_b=100, low_s_b=126, low_v_b=60;
int high_h_b=121, high_s_b=255, high_v_b=255;

// void on_low_h_thresh_trackbar_blue_1(int, void *);
// void on_high_h_thresh_trackbar_blue_1(int, void *);
// void on_low_s_thresh_trackbar_blue_1(int, void *);
// void on_high_s_thresh_trackbar_blue_1(int, void *);
// void on_low_v_thresh_trackbar_blue_1(int, void *);
// void on_high_v_thresh_trackbar_blue_1(int, void *);

// Initialization of variable for camera calibration paramters.

float intrinsic_data[9] = {1206.8897719532354, 0.0, 960.5, 0.0, 1206.8897719532354, 540.5, 0.0, 0.0, 1.0};
float distortion_data[5] = {0, 0, 0, 0, 0};

// Initialization of variable for text drawing
String text;
int iMin_tracking_ball_size = 5; // Minimum ball radius size by pixels. If ball is smaller than this, it won't be searched.
float fball_diameter = 0.14 ; // Initialization of variable for dimension of the target(real ball diameter by meter)

// Setting Mat variables for images.
Mat buffer;
Mat result;

// Setting Publishers
ros::Publisher upper_pub;
ros::Publisher upper_pubb;
ros::Publisher upper_pubr;
ros::Publisher upper_pubg;

ros::Publisher lower_pubb;
ros::Publisher lower_pubr;

ros::Publisher rear_pubr;
ros::Publisher rear_pubb;
ros::Publisher rear_pubg;

ros::Publisher rear_up_pubr;
ros::Publisher rear_up_pubb;
ros::Publisher rear_up_pubg;

ros::Publisher pub_markers;

// Declaring functions for image erode and dilaation.
void morphOps(Mat &thresh) {
  Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
  Mat dilateElement = getStructuringElement( MORPH_RECT,Size(3,3));
  morphologyEx(thresh, thresh, MORPH_CLOSE, erodeElement);
  morphologyEx(thresh, thresh, MORPH_OPEN, erodeElement);
}

// Declaration of functions that calculates the ball position from pixel position.
vector<float> pixel2point(Point center, int radius) {
  vector<float> position;
  float x, y, u, v, Xc, Yc, Zc;
  x = center.x;//.x;// .at(0);
  y = center.y;//.y;//
  u = (x-intrinsic_data[2])/intrinsic_data[0];
  v = (y-intrinsic_data[5])/intrinsic_data[4];
  Zc = (intrinsic_data[0]*fball_diameter)/(2*(float)radius) ;
  Xc = u*Zc ;
  Yc = v*Zc ;
  Xc = roundf(Xc * 1000) / 1000;
  Yc = roundf(Yc * 1000) / 1000;
  Zc = roundf(Zc * 1000) / 1000;
  position.push_back(Xc);
  position.push_back(Yc);
  position.push_back(Zc);
  return position;
}

// Changing int variable to string.
string intToString(int n) {
  stringstream s;
  s << n;
  return s.str();
}

// Changing float variable to string.
string floatToString(float f) {
  ostringstream buffer;
  buffer << f;
  return buffer.str();
}

void upper_ball_detect()
{
  // Declare intrinsic and distortions by using the variable declared before.
  Mat intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
  Mat distCoeffs = Mat(1, 5, CV_32F, distortion_data);

  // Declare another Mat variable to keep the image.
  Mat frame;
  frame = buffer;

  Mat calibrated_frame;
  // Undistort frame images and save to calibrated frame.
  undistort(frame, calibrated_frame, intrinsic, distCoeffs);

  // Defining Mat variables for Threshold images.

  Mat hsv_frame;
  Mat hsv_frame_red;
  Mat hsv_frame_red1;
  Mat hsv_frame_red2;
  Mat hsv_frame_blue;

  Mat hsv_frame_green;

  // Making clone of original image for drawing circles.
  result=calibrated_frame.clone();

  //Change RGB frame to HSV frame
  cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);

  //Threshold
  inRange(hsv_frame, Scalar(low_h_r,low_s_r,low_v_r), Scalar(high_h_r,high_s_r,high_v_r), hsv_frame_red1);
  inRange(hsv_frame, Scalar(low_h2_r,low_s_r,low_v_r), Scalar(high_h2_r,high_s_r,high_v_r), hsv_frame_red2);
  inRange(hsv_frame, Scalar(low_h_b,low_s_b,low_v_b), Scalar(high_h_b,high_s_b,high_v_b), hsv_frame_blue);
  inRange(hsv_frame, Scalar(low_h_g,low_s_g,low_v_g), Scalar(high_h_g,high_s_b,high_v_b), hsv_frame_green);
  addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);


  //Blur and erode, dilate
  Mat erodeElement = getStructuringElement( MORPH_RECT,Size(5,5));
  Mat hsv_frame_red_1, hsv_frame_red_2, hsv_frame_blue_1, hsv_frame_blue_2, hsv_frame_green_1, hsv_frame_green_2;

  morphologyEx(hsv_frame_red, hsv_frame_red_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_red_1, hsv_frame_red_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_red_2, hsv_frame_red, cv::Size(9, 9), 2, 2);

  morphologyEx(hsv_frame_blue, hsv_frame_blue_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_blue_1, hsv_frame_blue_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_blue_2, hsv_frame_blue, cv::Size(9, 9), 2, 2);

  morphologyEx(hsv_frame_green, hsv_frame_green_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_green_1, hsv_frame_green_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_green_2, hsv_frame_green, cv::Size(9, 9), 2, 2);

  //Canny Edge Detection
  int lowThreshold=100;
  int ratio=3;
  int kernel_size=3;

  Mat img_canny_blue;
  Mat img_canny_red;
  Mat img_canny_green;

  Canny(hsv_frame_blue, img_canny_blue, lowThreshold, lowThreshold*ratio, kernel_size);
  Canny(hsv_frame_red, img_canny_red, lowThreshold, lowThreshold*ratio, kernel_size);
  Canny(hsv_frame_green, img_canny_green, lowThreshold, lowThreshold*ratio, kernel_size);

  //Finding Contours for blue threshold image
  vector<Vec4i> hierarchy_b;
  vector<vector<Point> > contours_b;
  findContours(img_canny_blue, contours_b, hierarchy_b, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

  vector<Vec4i> hierarchy_r;
  vector<vector<Point> > contours_r;
  findContours(img_canny_red, contours_r, hierarchy_r, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

  vector<Vec4i> hierarchy_g;
  vector<vector<Point> > contours_g;
  findContours(img_canny_green, contours_g, hierarchy_g, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));


  // Define variables for contour poly, center of circles, radius of circles
  vector<vector<Point> > contours_poly_b( contours_b.size() );
  vector<Point2f> center_b( contours_b.size() );
  vector<float> radius_b( contours_b.size() );

  vector<vector<Point> > contours_poly_r( contours_r.size() );
  vector<Point2f> center_r( contours_r.size() );
  vector<float> radius_r( contours_r.size() );

  vector<vector<Point> > contours_poly_g( contours_g.size() );
  vector<Point2f> center_g( contours_g.size() );
  vector<float> radius_g( contours_g.size() );


  /*Finding blue balls by contours
    Find polygon from contours and find the minimun size enclosing circle of that polygon.
  */

  for( size_t i = 0; i < contours_b.size(); i++ )
  {
    approxPolyDP( contours_b[i], contours_poly_b[i], 1, true );
    minEnclosingCircle( contours_poly_b[i], center_b[i], radius_b[i] );
  }

  for( size_t i = 0; i < contours_r.size(); i++ )
  {
    approxPolyDP( contours_r[i], contours_poly_r[i], 1, true );
    minEnclosingCircle( contours_poly_r[i], center_r[i], radius_r[i] );
  }

  for( size_t i = 0; i < contours_g.size(); i++ )
  {
    approxPolyDP( contours_g[i], contours_poly_g[i], 1, true );
    minEnclosingCircle( contours_poly_g[i], center_g[i], radius_g[i] );
  }

  // Declare message variable to publish
  core_msgs::ball_position upper_msg;
  core_msgs::ball_position upper_msgb;
  core_msgs::ball_position upper_msgr;
  core_msgs::ball_position upper_msgg;

  core_msgs::ball_position_z lower_msgb;
  core_msgs::ball_position_z lower_msgr;

  int ball_num = 0;
  int ball_num_blue = 0;
  int ball_num_red = 0;
  int ball_num_green = 0;

  for (size_t i=0; i<contours_b.size(); i++)
  {
    if(radius_b[i] > iMin_tracking_ball_size)
    {
      //declare colors. Scalar(blue, green, red)
      Scalar color = Scalar(255,0,0);
      Scalar color_g = Scalar(0,255,0);

      // find the pixel point of the circle cneter, and the pixel radius of an circle

      float px_b = center_b[i].x;
      float py_b = center_b[i].y;
      float pr_b = radius_b[i];

      // change the pixel value to real world value

      vector<float> ball_pos_b;
      ball_pos_b = pixel2point(center_b[i],radius_b[i]);

      //draw the circle at the result Mat matrix
      //putText puts text at the matrix, puts text, at the point of an image

      float isx = ball_pos_b[0];
      float isy = ball_pos_b[1];
      float isz = ball_pos_b[2];

      string sx = floatToString(isx);
      string sy = floatToString(isy);
      string sz = floatToString(isz);

      string text;
      text = "x: " + sx +", y: " + sy + ", z: " + sz;
      putText(result, text, center_b[i], 2, 1, color_g, 2);
      circle(result, center_b[i], (int)radius_b[i], color, 2, 8, 0);
      ball_num = ball_num + 1;
      ball_num_blue = ball_num_blue + 1;

      //push back variables of real ball position to the message variable
      upper_msgb.img_x.push_back(ball_pos_b[0]);
      upper_msgb.img_y.push_back(ball_pos_b[2]);
      upper_msg.img_x.push_back(ball_pos_b[0]);
      upper_msg.img_y.push_back(ball_pos_b[2]);
    }
    upper_msgb.size = ball_num_blue;
  }

   // do same procedure for green balls

  for (size_t i=0; i<contours_g.size(); i++)
  {
    if(radius_g[i] > iMin_tracking_ball_size)
    {
      //declare colors. Scalar(blue, green, red)
      Scalar color = Scalar(0,255,0);
      Scalar color_g = Scalar(0,255,0);

      // find the pixel point of the circle cneter, and the pixel radius of an circle

      float px_g = center_g[i].x;
      float py_g = center_g[i].y;
      float pr_g = radius_g[i];

      // change the pixel value to real world value

      vector<float> ball_pos_g;
      ball_pos_g = pixel2point(center_g[i],radius_g[i]);

      //draw the circle at the result Mat matrix
      //putText puts text at the matrix, puts text, at the point of an image

      float isx = ball_pos_g[0];
      float isy = ball_pos_g[1];
      float isz = ball_pos_g[2];

      string sx = floatToString(isx);
      string sy = floatToString(isy);
      string sz = floatToString(isz);

      string text;
      text = "x: " + sx +", y: " + sy + ", z: " + sz;
      putText(result, text, center_g[i], 2, 1, color_g, 2);
      circle(result, center_g[i], (int)radius_g[i], color, 2, 8, 0);
      ball_num = ball_num + 1;
      ball_num_green = ball_num_green + 1;

      //push back variables of real ball position to the message variable
      upper_msgg.img_x.push_back(ball_pos_g[0]);
      upper_msgg.img_y.push_back(ball_pos_g[2]);
      upper_msg.img_x.push_back(ball_pos_g[0]);
      upper_msg.img_y.push_back(ball_pos_g[2]);
    }
    upper_msgg.size = ball_num_green;
  }


  // do same procedure for red balls

  for (size_t i=0; i<contours_r.size(); i++)
  {
    if(radius_r[i] > iMin_tracking_ball_size)
    {
      Scalar color = Scalar(0,0,255);
      Scalar color_g = Scalar(0,255,0);

      float px_r = center_r[i].x;
      float py_r = center_r[i].y;
      float pr_r = radius_r[i];

      vector<float> ball_pos_r;
      ball_pos_r = pixel2point(center_r[i],radius_r[i]);


      float isx = ball_pos_r[0];
      float isy = ball_pos_r[1];
      float isz = ball_pos_r[2];

      string sx = floatToString(isx);
      string sy = floatToString(isy);
      string sz = floatToString(isz);

      string text;
      text = "x: " + sx +", y: " + sy + ", z: " + sz;
      putText(result, text, center_r[i], 2, 1, color_g, 2);
      circle(result, center_r[i], (int)radius_r[i], color, 2, 8, 0);
      ball_num = ball_num + 1;
      ball_num_red = ball_num_red + 1;
      upper_msgr.img_x.push_back(ball_pos_r[0]);
      upper_msgr.img_y.push_back(ball_pos_r[2]);
      upper_msg.img_x.push_back(ball_pos_r[0]);
      upper_msg.img_y.push_back(ball_pos_r[2]);
    }
    upper_msgr.size = ball_num_red;
  }
  upper_msg.size = ball_num;
  //show what is published at the terminal
  //cout<<msg.size<<endl;
  //cout<<"Number of total balls in the map: " << ball_num << endl;
  //cout<<"Number of blue balls in the map: " << ball_num_blue << endl;
  //cout<<"Number of green balls in the map: " << ball_num_green << endl;
  //cout<<"Number of red balls in the map: " << ball_num_red << endl;
  //for (int i=0 ; i<ball_num ; i++)
  //{
    //cout<<msg.img_x[i]<<endl;
    //cout<<msg.img_y[i]<<endl;
  //}

  upper_pubg.publish(upper_msgg); //publish a message
  upper_pubr.publish(upper_msgr); //publish a message
  upper_pubb.publish(upper_msgb); //publish a message
  upper_pub.publish(upper_msg); //publish a message

  resizeWindow("upper", 480, 270);
  imshow("upper", result);
}

void lower_ball_detect()
{
  // Declare intrinsic and distortions by using the variable declared before.
  Mat intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
  Mat distCoeffs = Mat(1, 5, CV_32F, distortion_data);

  // Declare another Mat variable to keep the image.
  Mat frame;
  frame = buffer;

  Mat calibrated_frame;
  // Undistort frame images and save to calibrated frame.
  undistort(frame, calibrated_frame, intrinsic, distCoeffs);

  // Defining Mat variables for Threshold images.

  Mat hsv_frame;
  Mat hsv_frame_red;
  Mat hsv_frame_red1;
  Mat hsv_frame_red2;
  Mat hsv_frame_blue;

  Mat hsv_frame_green;

  // Making clone of original image for drawing circles.
  result = calibrated_frame.clone();

  //Change RGB frame to HSV frame
  cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);

  //Threshold
  inRange(hsv_frame, Scalar(low_h_r,low_s_r,low_v_r), Scalar(high_h_r,high_s_r,high_v_r), hsv_frame_red1);
  inRange(hsv_frame, Scalar(low_h2_r,low_s_r,low_v_r), Scalar(high_h2_r,high_s_r,high_v_r), hsv_frame_red2);
  inRange(hsv_frame, Scalar(low_h_b,low_s_b,low_v_b), Scalar(high_h_b,high_s_b,high_v_b), hsv_frame_blue);
  inRange(hsv_frame, Scalar(low_h_g,low_s_g,low_v_g), Scalar(high_h_g,high_s_b,high_v_b), hsv_frame_green);
  addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);


  //Blur and erode, dilate
  Mat erodeElement = getStructuringElement( MORPH_RECT,Size(5,5));
  Mat hsv_frame_red_1, hsv_frame_red_2, hsv_frame_blue_1, hsv_frame_blue_2, hsv_frame_green_1, hsv_frame_green_2;

  morphologyEx(hsv_frame_red, hsv_frame_red_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_red_1, hsv_frame_red_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_red_2, hsv_frame_red, cv::Size(9, 9), 2, 2);

  morphologyEx(hsv_frame_blue, hsv_frame_blue_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_blue_1, hsv_frame_blue_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_blue_2, hsv_frame_blue, cv::Size(9, 9), 2, 2);

  morphologyEx(hsv_frame_green, hsv_frame_green_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_green_1, hsv_frame_green_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_green_2, hsv_frame_green, cv::Size(9, 9), 2, 2);

  //Canny Edge Detection
  int lowThreshold=100;
  int ratio=3;
  int kernel_size=3;

  Mat img_canny_blue;
  Mat img_canny_red;
  Mat img_canny_green;

  Canny(hsv_frame_blue, img_canny_blue, lowThreshold, lowThreshold*ratio, kernel_size);
  Canny(hsv_frame_red, img_canny_red, lowThreshold, lowThreshold*ratio, kernel_size);
  Canny(hsv_frame_green, img_canny_green, lowThreshold, lowThreshold*ratio, kernel_size);

  //Finding Contours for blue threshold image
  vector<Vec4i> hierarchy_b;
  vector<vector<Point> > contours_b;
  findContours(img_canny_blue, contours_b, hierarchy_b, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

  vector<Vec4i> hierarchy_r;
  vector<vector<Point> > contours_r;
  findContours(img_canny_red, contours_r, hierarchy_r, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

  vector<Vec4i> hierarchy_g;
  vector<vector<Point> > contours_g;
  findContours(img_canny_green, contours_g, hierarchy_g, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));


  // Define variables for contour poly, center of circles, radius of circles
  vector<vector<Point> > contours_poly_b( contours_b.size() );
  vector<Point2f> center_b( contours_b.size() );
  vector<float> radius_b( contours_b.size() );

  vector<vector<Point> > contours_poly_r( contours_r.size() );
  vector<Point2f> center_r( contours_r.size() );
  vector<float> radius_r( contours_r.size() );

  vector<vector<Point> > contours_poly_g( contours_g.size() );
  vector<Point2f> center_g( contours_g.size() );
  vector<float> radius_g( contours_g.size() );


  /*Finding blue balls by contours
    Find polygon from contours and find the minimun size enclosing circle of that polygon.
  */

  for( size_t i = 0; i < contours_b.size(); i++ )
  {
    approxPolyDP( contours_b[i], contours_poly_b[i], 1, true );
    minEnclosingCircle( contours_poly_b[i], center_b[i], radius_b[i] );
  }

  for( size_t i = 0; i < contours_r.size(); i++ )
  {
    approxPolyDP( contours_r[i], contours_poly_r[i], 1, true );
    minEnclosingCircle( contours_poly_r[i], center_r[i], radius_r[i] );
  }

  for( size_t i = 0; i < contours_g.size(); i++ )
  {
    approxPolyDP( contours_g[i], contours_poly_g[i], 1, true );
    minEnclosingCircle( contours_poly_g[i], center_g[i], radius_g[i] );
  }

  // Declare message variable to publish

  core_msgs::ball_position_z lower_msgb;
  core_msgs::ball_position_z lower_msgr;

  int ball_num_blue = 0;
  int ball_num_red = 0;

  for (size_t i=0; i<contours_b.size(); i++)
  {
    if(radius_b[i] > iMin_tracking_ball_size)
    {
      //declare colors. Scalar(blue, green, red)
      Scalar color = Scalar(255,0,0);
      Scalar color_g = Scalar(0,255,0);

      // find the pixel point of the circle cneter, and the pixel radius of an circle

      float px_b = center_b[i].x;
      float py_b = center_b[i].y;
      float pr_b = radius_b[i];

      // change the pixel value to real world value

      vector<float> ball_pos_b;
      ball_pos_b = pixel2point(center_b[i],radius_b[i]);

      //draw the circle at the result Mat matrix
      //putText puts text at the matrix, puts text, at the point of an image

      float isx = ball_pos_b[0];
      float isy = ball_pos_b[1];
      float isz = ball_pos_b[2];

      string sx = floatToString(isx);
      string sy = floatToString(isy);
      string sz = floatToString(isz);

      string text;
      text = "x: " + sx +", y: " + sy + ", z: " + sz;
      putText(result, text, center_b[i], 2, 1, color_g, 2);
      circle(result, center_b[i], (int)radius_b[i], color, 2, 8, 0);
      ball_num_blue = ball_num_blue + 1;

      //push back variables of real ball position to the message variable
      lower_msgb.img_x.push_back(ball_pos_b[0]);
      lower_msgb.img_y.push_back(ball_pos_b[2]);
      lower_msgb.img_z.push_back(ball_pos_b[1]);
    }
    lower_msgb.size = ball_num_blue;
  }

  // do same procedure for red balls

  for (size_t i=0; i<contours_r.size(); i++)
  {
    if(radius_r[i] > iMin_tracking_ball_size)
    {
      Scalar color = Scalar(0,0,255);
      Scalar color_g = Scalar(0,255,0);

      float px_r = center_r[i].x;
      float py_r = center_r[i].y;
      float pr_r = radius_r[i];

      vector<float> ball_pos_r;
      ball_pos_r = pixel2point(center_r[i],radius_r[i]);


      float isx = ball_pos_r[0];
      float isy = ball_pos_r[1];
      float isz = ball_pos_r[2];

      string sx = floatToString(isx);
      string sy = floatToString(isy);
      string sz = floatToString(isz);

      string text;
      text = "x: " + sx +", y: " + sy + ", z: " + sz;
      putText(result, text, center_r[i], 2, 1, color_g, 2);
      circle(result, center_r[i], (int)radius_r[i], color, 2, 8, 0);
      ball_num_red = ball_num_red + 1;

      lower_msgr.img_x.push_back(ball_pos_r[0]);
      lower_msgr.img_y.push_back(ball_pos_r[2]);
      lower_msgr.img_z.push_back(ball_pos_r[1]);
    }
    lower_msgr.size = ball_num_red;
  }
  //msg.size = ball_num;
  //show what is published at the terminal
  //cout<<msg.size<<endl;
  //cout<<"Number of blue balls in the lower camera: " << ball_num_blue << endl;
  //cout<<"Number of red balls in the lower camera: " << ball_num_red << endl;
  //for (int i=0 ; i<ball_num ; i++)
  //{
    //cout<<msg.img_x[i]<<endl;
    //cout<<msg.img_y[i]<<endl;
  //}

  lower_pubr.publish(lower_msgr); //publish a message
  lower_pubb.publish(lower_msgb); //publish a message

  resizeWindow("lower", 480,270);

  imshow("lower", result);
}

void rear_ball_detect()
{
  // Declare intrinsic and distortions by using the variable declared before.
  Mat intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
  Mat distCoeffs = Mat(1, 5, CV_32F, distortion_data);

  // Declare another Mat variable to keep the image.
  Mat frame;
  frame = buffer;

  Mat calibrated_frame;
  // Undistort frame images and save to calibrated frame.
  undistort(frame, calibrated_frame, intrinsic, distCoeffs);

  // Defining Mat variables for Threshold images.

  Mat hsv_frame;
  Mat hsv_frame_red;
  Mat hsv_frame_red1;
  Mat hsv_frame_red2;
  Mat hsv_frame_blue;

  Mat hsv_frame_green;

  // Making clone of original image for drawing circles.
  result = calibrated_frame.clone();

  //Change RGB frame to HSV frame
  cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);

  //Threshold
  inRange(hsv_frame, Scalar(low_h_r,low_s_r,low_v_r), Scalar(high_h_r,high_s_r,high_v_r), hsv_frame_red1);
  inRange(hsv_frame, Scalar(low_h2_r,low_s_r,low_v_r), Scalar(high_h2_r,high_s_r,high_v_r), hsv_frame_red2);
  inRange(hsv_frame, Scalar(low_h_b,low_s_b,low_v_b), Scalar(high_h_b,high_s_b,high_v_b), hsv_frame_blue);
  inRange(hsv_frame, Scalar(low_h_g,low_s_g,low_v_g), Scalar(high_h_g,high_s_b,high_v_b), hsv_frame_green);
  addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);


  //Blur and erode, dilate
  Mat erodeElement = getStructuringElement( MORPH_RECT,Size(5,5));
  Mat hsv_frame_red_1, hsv_frame_red_2, hsv_frame_blue_1, hsv_frame_blue_2, hsv_frame_green_1, hsv_frame_green_2;

  morphologyEx(hsv_frame_red, hsv_frame_red_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_red_1, hsv_frame_red_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_red_2, hsv_frame_red, cv::Size(9, 9), 2, 2);

  morphologyEx(hsv_frame_blue, hsv_frame_blue_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_blue_1, hsv_frame_blue_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_blue_2, hsv_frame_blue, cv::Size(9, 9), 2, 2);

  morphologyEx(hsv_frame_green, hsv_frame_green_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_green_1, hsv_frame_green_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_green_2, hsv_frame_green, cv::Size(9, 9), 2, 2);

  //Canny Edge Detection
  int rearThreshold=100;
  int ratio=3;
  int kernel_size=3;

  Mat img_canny_blue;
  Mat img_canny_red;
  Mat img_canny_green;

  Canny(hsv_frame_blue, img_canny_blue, rearThreshold, rearThreshold*ratio, kernel_size);
  Canny(hsv_frame_red, img_canny_red, rearThreshold, rearThreshold*ratio, kernel_size);
  Canny(hsv_frame_green, img_canny_green, rearThreshold, rearThreshold*ratio, kernel_size);

  //Finding Contours for blue threshold image
  vector<Vec4i> hierarchy_b;
  vector<vector<Point> > contours_b;
  findContours(img_canny_blue, contours_b, hierarchy_b, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

  vector<Vec4i> hierarchy_r;
  vector<vector<Point> > contours_r;
  findContours(img_canny_red, contours_r, hierarchy_r, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

  vector<Vec4i> hierarchy_g;
  vector<vector<Point> > contours_g;
  findContours(img_canny_green, contours_g, hierarchy_g, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));


  // Define variables for contour poly, center of circles, radius of circles
  vector<vector<Point> > contours_poly_b( contours_b.size() );
  vector<Point2f> center_b( contours_b.size() );
  vector<float> radius_b( contours_b.size() );

  vector<vector<Point> > contours_poly_r( contours_r.size() );
  vector<Point2f> center_r( contours_r.size() );
  vector<float> radius_r( contours_r.size() );

  vector<vector<Point> > contours_poly_g( contours_g.size() );
  vector<Point2f> center_g( contours_g.size() );
  vector<float> radius_g( contours_g.size() );


  /*Finding blue balls by contours
    Find polygon from contours and find the minimun size enclosing circle of that polygon.
  */

  for( size_t i = 0; i < contours_b.size(); i++ )
  {
    approxPolyDP( contours_b[i], contours_poly_b[i], 1, true );
    minEnclosingCircle( contours_poly_b[i], center_b[i], radius_b[i] );
  }

  for( size_t i = 0; i < contours_r.size(); i++ )
  {
    approxPolyDP( contours_r[i], contours_poly_r[i], 1, true );
    minEnclosingCircle( contours_poly_r[i], center_r[i], radius_r[i] );
  }

  for( size_t i = 0; i < contours_g.size(); i++ )
  {
    approxPolyDP( contours_g[i], contours_poly_g[i], 1, true );
    minEnclosingCircle( contours_poly_g[i], center_g[i], radius_g[i] );
  }

  // Declare message variable to publish

  core_msgs::ball_position rear_msgb;
  core_msgs::ball_position rear_msgr;
  core_msgs::ball_position rear_msgg;

  int ball_num = 0;
  int ball_num_blue = 0;
  int ball_num_red = 0;
  int ball_num_green = 0;

  for (size_t i=0; i<contours_b.size(); i++)
  {
    if(radius_b[i] > iMin_tracking_ball_size)
    {
      //declare colors. Scalar(blue, green, red)
      Scalar color = Scalar(255,0,0);
      Scalar color_g = Scalar(0,255,0);

      // find the pixel point of the circle cneter, and the pixel radius of an circle

      float px_b = center_b[i].x;
      float py_b = center_b[i].y;
      float pr_b = radius_b[i];

      // change the pixel value to real world value

      vector<float> ball_pos_b;
      ball_pos_b = pixel2point(center_b[i],radius_b[i]);

      //draw the circle at the result Mat matrix
      //putText puts text at the matrix, puts text, at the point of an image

      float isx = ball_pos_b[0];
      float isy = ball_pos_b[1];
      float isz = ball_pos_b[2];

      string sx = floatToString(isx);
      string sy = floatToString(isy);
      string sz = floatToString(isz);

      string text;
      text = "x: " + sx +", y: " + sy + ", z: " + sz;
      putText(result, text, center_b[i], 2, 1, color_g, 2);
      circle(result, center_b[i], (int)radius_b[i], color, 2, 8, 0);
      ball_num_blue = ball_num_blue + 1;

      //push back variables of real ball position to the message variable
      rear_msgb.img_x.push_back(ball_pos_b[0]);
      rear_msgb.img_y.push_back(ball_pos_b[2]);
    }
    rear_msgb.size = ball_num_blue;
  }

  // do same procedure for red balls
  for (size_t i=0; i<contours_g.size(); i++)
  {
    if(radius_g[i] > iMin_tracking_ball_size)
    {
      //declare colors. Scalar(blue, green, red)
      Scalar color = Scalar(0,255,0);
      Scalar color_g = Scalar(0,255,0);

      // find the pixel point of the circle cneter, and the pixel radius of an circle

      float px_g = center_g[i].x;
      float py_g = center_g[i].y;
      float pr_g = radius_g[i];

      // change the pixel value to real world value

      vector<float> ball_pos_g;
      ball_pos_g = pixel2point(center_g[i],radius_g[i]);

      //draw the circle at the result Mat matrix
      //putText puts text at the matrix, puts text, at the point of an image

      float isx = ball_pos_g[0];
      float isy = ball_pos_g[1];
      float isz = ball_pos_g[2];

      string sx = floatToString(isx);
      string sy = floatToString(isy);
      string sz = floatToString(isz);

      string text;
      text = "x: " + sx +", y: " + sy + ", z: " + sz;
      putText(result, text, center_g[i], 2, 1, color_g, 2);
      circle(result, center_g[i], (int)radius_g[i], color, 2, 8, 0);
      ball_num = ball_num + 1;
      ball_num_green = ball_num_green + 1;

      //push back variables of real ball position to the message variable
      rear_msgg.img_x.push_back(ball_pos_g[0]);
      rear_msgg.img_y.push_back(ball_pos_g[2]);
    }
    rear_msgg.size = ball_num_green;
  }

  for (size_t i=0; i<contours_r.size(); i++)
  {
    if(radius_r[i] > iMin_tracking_ball_size)
    {
      Scalar color = Scalar(0,0,255);
      Scalar color_g = Scalar(0,255,0);

      float px_r = center_r[i].x;
      float py_r = center_r[i].y;
      float pr_r = radius_r[i];

      vector<float> ball_pos_r;
      ball_pos_r = pixel2point(center_r[i],radius_r[i]);


      float isx = ball_pos_r[0];
      float isy = ball_pos_r[1];
      float isz = ball_pos_r[2];

      string sx = floatToString(isx);
      string sy = floatToString(isy);
      string sz = floatToString(isz);

      string text;
      text = "x: " + sx +", y: " + sy + ", z: " + sz;
      putText(result, text, center_r[i], 2, 1, color_g, 2);
      circle(result, center_r[i], (int)radius_r[i], color, 2, 8, 0);
      ball_num_red = ball_num_red + 1;

      rear_msgr.img_x.push_back(ball_pos_r[0]);
      rear_msgr.img_y.push_back(ball_pos_r[2]);
    }
    rear_msgr.size = ball_num_red;
  }
  //msg.size = ball_num;
  //show what is published at the terminal
  //cout<<msg.size<<endl;
  //cout<<"Number of blue balls in the rear camera: " << ball_num_blue << endl;
  //cout<<"Number of red balls in the rear camera: " << ball_num_red << endl;
  //for (int i=0 ; i<ball_num ; i++)
  //{
    //cout<<msg.img_x[i]<<endl;
    //cout<<msg.img_y[i]<<endl;
  //}
  rear_pubg.publish(rear_msgg);
  rear_pubr.publish(rear_msgr); //publish a message
  rear_pubb.publish(rear_msgb); //publish a message

  resizeWindow("rear", 480,270);

  imshow("rear", result);
}

void rear_up_ball_detect()
{
  // Declare intrinsic and distortions by using the variable declared before.
  Mat intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
  Mat distCoeffs = Mat(1, 5, CV_32F, distortion_data);

  // Declare another Mat variable to keep the image.
  Mat frame;
  frame = buffer;

  Mat calibrated_frame;
  // Undistort frame images and save to calibrated frame.
  undistort(frame, calibrated_frame, intrinsic, distCoeffs);

  // Defining Mat variables for Threshold images.

  Mat hsv_frame;
  Mat hsv_frame_red;
  Mat hsv_frame_red1;
  Mat hsv_frame_red2;
  Mat hsv_frame_blue;

  Mat hsv_frame_green;

  // Making clone of original image for drawing circles.
  result=calibrated_frame.clone();

  //Change RGB frame to HSV frame
  cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);

  //Threshold
  inRange(hsv_frame, Scalar(low_h_r,low_s_r,low_v_r), Scalar(high_h_r,high_s_r,high_v_r), hsv_frame_red1);
  inRange(hsv_frame, Scalar(low_h2_r,low_s_r,low_v_r), Scalar(high_h2_r,high_s_r,high_v_r), hsv_frame_red2);
  inRange(hsv_frame, Scalar(low_h_b,low_s_b,low_v_b), Scalar(high_h_b,high_s_b,high_v_b), hsv_frame_blue);
  inRange(hsv_frame, Scalar(low_h_g,low_s_g,low_v_g), Scalar(high_h_g,high_s_b,high_v_b), hsv_frame_green);
  addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);


  //Blur and erode, dilate
  Mat erodeElement = getStructuringElement( MORPH_RECT,Size(5,5));
  Mat hsv_frame_red_1, hsv_frame_red_2, hsv_frame_blue_1, hsv_frame_blue_2, hsv_frame_green_1, hsv_frame_green_2;

  morphologyEx(hsv_frame_red, hsv_frame_red_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_red_1, hsv_frame_red_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_red_2, hsv_frame_red, cv::Size(9, 9), 2, 2);

  morphologyEx(hsv_frame_blue, hsv_frame_blue_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_blue_1, hsv_frame_blue_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_blue_2, hsv_frame_blue, cv::Size(9, 9), 2, 2);

  morphologyEx(hsv_frame_green, hsv_frame_green_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_green_1, hsv_frame_green_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_green_2, hsv_frame_green, cv::Size(9, 9), 2, 2);

  //Canny Edge Detection
  int lowThreshold=100;
  int ratio=3;
  int kernel_size=3;

  Mat img_canny_blue;
  Mat img_canny_red;
  Mat img_canny_green;

  Canny(hsv_frame_blue, img_canny_blue, lowThreshold, lowThreshold*ratio, kernel_size);
  Canny(hsv_frame_red, img_canny_red, lowThreshold, lowThreshold*ratio, kernel_size);
  Canny(hsv_frame_green, img_canny_green, lowThreshold, lowThreshold*ratio, kernel_size);

  //Finding Contours for blue threshold image
  vector<Vec4i> hierarchy_b;
  vector<vector<Point> > contours_b;
  findContours(img_canny_blue, contours_b, hierarchy_b, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

  vector<Vec4i> hierarchy_r;
  vector<vector<Point> > contours_r;
  findContours(img_canny_red, contours_r, hierarchy_r, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

  vector<Vec4i> hierarchy_g;
  vector<vector<Point> > contours_g;
  findContours(img_canny_green, contours_g, hierarchy_g, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));


  // Define variables for contour poly, center of circles, radius of circles
  vector<vector<Point> > contours_poly_b( contours_b.size() );
  vector<Point2f> center_b( contours_b.size() );
  vector<float> radius_b( contours_b.size() );

  vector<vector<Point> > contours_poly_r( contours_r.size() );
  vector<Point2f> center_r( contours_r.size() );
  vector<float> radius_r( contours_r.size() );

  vector<vector<Point> > contours_poly_g( contours_g.size() );
  vector<Point2f> center_g( contours_g.size() );
  vector<float> radius_g( contours_g.size() );


  /*Finding blue balls by contours
    Find polygon from contours and find the minimun size enclosing circle of that polygon.
  */

  for( size_t i = 0; i < contours_b.size(); i++ )
  {
    approxPolyDP( contours_b[i], contours_poly_b[i], 1, true );
    minEnclosingCircle( contours_poly_b[i], center_b[i], radius_b[i] );
  }

  for( size_t i = 0; i < contours_r.size(); i++ )
  {
    approxPolyDP( contours_r[i], contours_poly_r[i], 1, true );
    minEnclosingCircle( contours_poly_r[i], center_r[i], radius_r[i] );
  }

  for( size_t i = 0; i < contours_g.size(); i++ )
  {
    approxPolyDP( contours_g[i], contours_poly_g[i], 1, true );
    minEnclosingCircle( contours_poly_g[i], center_g[i], radius_g[i] );
  }

  // Declare message variable to publish
  core_msgs::ball_position rear_up_msgb;
  core_msgs::ball_position rear_up_msgr;
  core_msgs::ball_position rear_up_msgg;

  int ball_num = 0;
  int ball_num_blue = 0;
  int ball_num_red = 0;
  int ball_num_green = 0;

  for (size_t i=0; i<contours_b.size(); i++)
  {
    if(radius_b[i] > iMin_tracking_ball_size)
    {
      //declare colors. Scalar(blue, green, red)
      Scalar color = Scalar(255,0,0);
      Scalar color_g = Scalar(0,255,0);

      // find the pixel point of the circle cneter, and the pixel radius of an circle

      float px_b = center_b[i].x;
      float py_b = center_b[i].y;
      float pr_b = radius_b[i];

      // change the pixel value to real world value

      vector<float> ball_pos_b;
      ball_pos_b = pixel2point(center_b[i],radius_b[i]);

      //draw the circle at the result Mat matrix
      //putText puts text at the matrix, puts text, at the point of an image

      float isx = ball_pos_b[0];
      float isy = ball_pos_b[1];
      float isz = ball_pos_b[2];

      string sx = floatToString(isx);
      string sy = floatToString(isy);
      string sz = floatToString(isz);

      string text;
      text = "x: " + sx +", y: " + sy + ", z: " + sz;
      putText(result, text, center_b[i], 2, 1, color_g, 2);
      circle(result, center_b[i], (int)radius_b[i], color, 2, 8, 0);
      ball_num = ball_num + 1;
      ball_num_blue = ball_num_blue + 1;

      //push back variables of real ball position to the message variable
      rear_up_msgb.img_x.push_back(ball_pos_b[0]);
      rear_up_msgb.img_y.push_back(ball_pos_b[2]);
    }
    rear_up_msgb.size = ball_num_blue;
  }

   // do same procedure for green balls

  for (size_t i=0; i<contours_g.size(); i++)
  {
    if(radius_g[i] > iMin_tracking_ball_size)
    {
      //declare colors. Scalar(blue, green, red)
      Scalar color = Scalar(0,255,0);
      Scalar color_g = Scalar(0,255,0);

      // find the pixel point of the circle cneter, and the pixel radius of an circle

      float px_g = center_g[i].x;
      float py_g = center_g[i].y;
      float pr_g = radius_g[i];

      // change the pixel value to real world value

      vector<float> ball_pos_g;
      ball_pos_g = pixel2point(center_g[i],radius_g[i]);

      //draw the circle at the result Mat matrix
      //putText puts text at the matrix, puts text, at the point of an image

      float isx = ball_pos_g[0];
      float isy = ball_pos_g[1];
      float isz = ball_pos_g[2];

      string sx = floatToString(isx);
      string sy = floatToString(isy);
      string sz = floatToString(isz);

      string text;
      text = "x: " + sx +", y: " + sy + ", z: " + sz;
      putText(result, text, center_g[i], 2, 1, color_g, 2);
      circle(result, center_g[i], (int)radius_g[i], color, 2, 8, 0);
      ball_num = ball_num + 1;
      ball_num_green = ball_num_green + 1;

      //push back variables of real ball position to the message variable
      rear_up_msgg.img_x.push_back(ball_pos_g[0]);
      rear_up_msgg.img_y.push_back(ball_pos_g[2]);
    }
    rear_up_msgg.size = ball_num_green;
  }


  // do same procedure for red balls

  for (size_t i=0; i<contours_r.size(); i++)
  {
    if(radius_r[i] > iMin_tracking_ball_size)
    {
      Scalar color = Scalar(0,0,255);
      Scalar color_g = Scalar(0,255,0);

      float px_r = center_r[i].x;
      float py_r = center_r[i].y;
      float pr_r = radius_r[i];

      vector<float> ball_pos_r;
      ball_pos_r = pixel2point(center_r[i],radius_r[i]);


      float isx = ball_pos_r[0];
      float isy = ball_pos_r[1];
      float isz = ball_pos_r[2];

      string sx = floatToString(isx);
      string sy = floatToString(isy);
      string sz = floatToString(isz);

      string text;
      text = "x: " + sx +", y: " + sy + ", z: " + sz;
      putText(result, text, center_r[i], 2, 1, color_g, 2);
      circle(result, center_r[i], (int)radius_r[i], color, 2, 8, 0);
      ball_num = ball_num + 1;
      ball_num_red = ball_num_red + 1;
      rear_up_msgr.img_x.push_back(ball_pos_r[0]);
      rear_up_msgr.img_y.push_back(ball_pos_r[2]);
    }
    rear_up_msgr.size = ball_num_red;
  }

  rear_up_pubg.publish(rear_up_msgg); //publish a message
  rear_up_pubr.publish(rear_up_msgr); //publish a message
  rear_up_pubb.publish(rear_up_msgb); //publish a message

  resizeWindow("rear_up", 480, 270);
  imshow("rear_up", result);
}

void upper_imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if(msg->height==480&&buffer.size().width==320) {  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
    std::cout<<"resized"<<std::endl;
    cv::resize(buffer,buffer,cv::Size(640,480));
  }
  try {buffer = cv_bridge::toCvShare(msg, "bgr8")->image;} //transfer the image data into buffer
  catch (cv_bridge::Exception& e) {ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());}
  upper_ball_detect();
  waitKey(3);
}

void lower_imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if(msg->height==480&&buffer.size().width==320) {  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
    std::cout<<"resized"<<std::endl;
    cv::resize(buffer,buffer,cv::Size(640,480));
  }
  try {buffer = cv_bridge::toCvShare(msg, "bgr8")->image;} //transfer the image data into buffer
  catch (cv_bridge::Exception& e) {ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());}
  lower_ball_detect();
  waitKey(3);
}

void rear_imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if(msg->height==480&&buffer.size().width==320) {  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
    std::cout<<"resized"<<std::endl;
    cv::resize(buffer,buffer,cv::Size(640,480));
  }
  try {buffer = cv_bridge::toCvShare(msg, "bgr8")->image;} //transfer the image data into buffer
  catch (cv_bridge::Exception& e) {ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());}
  rear_ball_detect();
  waitKey(3);
}

void rear_up_imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if(msg->height==480&&buffer.size().width==320) {  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
    std::cout<<"resized"<<std::endl;
    cv::resize(buffer,buffer,cv::Size(640,480));
  }
  try {buffer = cv_bridge::toCvShare(msg, "bgr8")->image;} //transfer the image data into buffer
  catch (cv_bridge::Exception& e) {ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());}
  rear_up_ball_detect();
  waitKey(3);
}

bool line_trace_over = false;
void line_trace_Callback(const std_msgs::Bool::ConstPtr& data) {
	line_trace_over = data->data;
}
int main(int argc, char **argv) {

 // Trackbars to set thresholds for HSV values : Blue ball
  // namedWindow("Object Detection_HSV_Blue1", WINDOW_NORMAL);
  // namedWindow("Object Detection_HSV_Red1", WINDOW_NORMAL);
  // moveWindow("Object Detection_HSV_Red1", 50,370);
  // moveWindow("Object Detection_HSV_Blue1", 50,370);
  // createTrackbar("Low H","Object Detection_HSV_Blue1", &low_h_b, 180, on_low_h_thresh_trackbar_blue_1);
  // createTrackbar("High H","Object Detection_HSV_Blue1", &high_h_b, 180, on_high_h_thresh_trackbar_blue_1);
  // createTrackbar("Low S","Object Detection_HSV_Blue1", &low_s_b, 255, on_low_s_thresh_trackbar_blue_1);
  // createTrackbar("High S","Object Detection_HSV_Blue1", &high_s_b, 255, on_high_s_thresh_trackbar_blue_1);
  // createTrackbar("Low V","Object Detection_HSV_Blue1", &low_v_b, 255, on_low_v_thresh_trackbar_blue_1);
  // createTrackbar("High V","Object Detection_HSV_Blue1", &high_v_b, 255, on_high_v_thresh_trackbar_blue_1);
  //
  //
  // createTrackbar("Low H","Object Detection_HSV_Red1", &low_h_r_1, 180, on_low_h_thresh_trackbar_red_1);
  // createTrackbar("High H","Object Detection_HSV_Red1", &high_h_r_1, 180, on_high_h_thresh_trackbar_red_1);
  // createTrackbar("Low H2","Object Detection_HSV_Red1", &low_h_r_2, 180, on_low_h2_thresh_trackbar_red_1);
  // createTrackbar("High H2","Object Detection_HSV_Red1", &high_h_r_2, 180, on_high_h2_thresh_trackbar_red_1);
  // createTrackbar("Low S","Object Detection_HSV_Red1", &low_s_r_1, 255, on_low_s_thresh_trackbar_red_1);
  // createTrackbar("High S","Object Detection_HSV_Red1", &high_s_r_1, 255, on_high_s_thresh_trackbar_red_1);
  // createTrackbar("Low V","Object Detection_HSV_Red1", &low_v_r_1, 255, on_low_v_thresh_trackbar_red_1);
  // createTrackbar("High V","Object Detection_HSV_Red1", &high_v_r_1, 255, on_high_v_thresh_trackbar_red_1);


  ros::init(argc, argv, "ball_detect_node"); //init ros nodd
  ros::NodeHandle nh; //create node handler
	ros::Subscriber sub_line_tracing_over = nh.subscribe<std_msgs::Bool>("/line_trace_over", 2, line_trace_Callback);
  ros::Rate loop_rate(20);
	while(!line_trace_over && ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
  image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
  image_transport::Subscriber upper_sub = it.subscribe("/upper_camera/rgb/image_raw", 1, upper_imageCallback);
  image_transport::Subscriber lower_sub = it.subscribe("/lower_camera/rgb/image_raw", 1, lower_imageCallback);
  image_transport::Subscriber rear_sub = it.subscribe("/rear_camera/rgb/image_raw", 1, rear_imageCallback);
  image_transport::Subscriber rear_up_sub = it.subscribe("/rear_up_camera/rgb/image_raw", 1, rear_up_imageCallback);
  upper_pub = nh.advertise<core_msgs::ball_position>("/position", 1); //setting publisher
  upper_pubb = nh.advertise<core_msgs::ball_position>("/position_blue", 1); //setting publisher
  upper_pubr = nh.advertise<core_msgs::ball_position>("/position_red", 1); //setting publisher
  upper_pubg = nh.advertise<core_msgs::ball_position>("/position_green", 1); //setting publisher
  lower_pubb = nh.advertise<core_msgs::ball_position_z>("/position_blue_lower", 1); //setting publisher
  lower_pubr = nh.advertise<core_msgs::ball_position_z>("/position_red_lower", 1); //setting publisher
  rear_pubr = nh.advertise<core_msgs::ball_position>("/position_red_rear", 1); //setting publisher
  rear_pubb = nh.advertise<core_msgs::ball_position>("/position_blue_rear", 1); //setting publisher
  rear_pubg = nh.advertise<core_msgs::ball_position>("/position_green_rear", 1); //setting publisher
  rear_up_pubr = nh.advertise<core_msgs::ball_position>("/position_rear_up_red", 1); //setting publisher
  rear_up_pubb = nh.advertise<core_msgs::ball_position>("/position_rear_up_blue", 1); //setting publisher
  rear_up_pubg = nh.advertise<core_msgs::ball_position>("/position_rear_up_green", 1);
  namedWindow("upper", WINDOW_NORMAL);
  namedWindow("lower", WINDOW_NORMAL);
  namedWindow("rear", WINDOW_NORMAL);
  namedWindow("rear_up", WINDOW_NORMAL);

  while(ros::ok()){
     ros::spinOnce();
     loop_rate.sleep();
  }

  return 0;
}

// void on_low_h_thresh_trackbar_blue_1(int, void *){
// low_h_b = min(high_h_b-1, low_h_b);
// setTrackbarPos("Low H","Object Detection_HSV_Blue1", low_h_b);
// }
// void on_high_h_thresh_trackbar_blue_1(int, void *){
// high_h_b = max(high_h_b, low_h_b+1);
// setTrackbarPos("High H", "Object Detection_HSV_Blue1", high_h_b);
// }
// void on_low_s_thresh_trackbar_blue_1(int, void *){
// low_s_b = min(high_s_b-1, low_s_b);
// setTrackbarPos("Low S","Object Detection_HSV_Blue1", low_s_b);
// }
// void on_high_s_thresh_trackbar_blue_1(int, void *){
// high_s_b = max(high_s_b, low_s_b+1);
// setTrackbarPos("High S", "Object Detection_HSV_Blue1", high_s_b);
// }
// void on_low_v_thresh_trackbar_blue_1(int, void *){
// low_v_b= min(high_v_b-1, low_v_b);
// setTrackbarPos("Low V","Object Detection_HSV_Blue1", low_v_b);
// }
// void on_high_v_thresh_trackbar_blue_1(int, void *){
// high_v_b = max(high_v_b, low_v_b+1);
// setTrackbarPos("High V", "Object Detection_HSV_Blue1", high_v_b);
// }
//
//
// void on_low_h_thresh_trackbar_red_1(int, void *){
// low_h_r_1 = min(high_h_r_1-1, low_h_r_1);
// setTrackbarPos("Low H","Object Detection_HSV_Red1", low_h_r_1);
// }
// void on_high_h_thresh_trackbar_red_1(int, void *){
// high_h_r_1 = max(high_h_r_1, low_h_r_1+1);
// setTrackbarPos("High H", "Object Detection_HSV_Red1", high_h_r_1);
// }
// void on_low_h2_thresh_trackbar_red_1(int, void *){
// low_h_r_2 = min(high_h_r_2-1, low_h_r_2);
// setTrackbarPos("Low H2","Object Detection_HSV_Red1", low_h_r_2);
// }
// void on_high_h2_thresh_trackbar_red_1(int, void *){
// high_h_r_2 = max(high_h_r_2, low_h_r_2+1);
// setTrackbarPos("High H2", "Object Detection_HSV_Red1", high_h_r_2);
// }
// void on_low_s_thresh_trackbar_red_1(int, void *){
// low_s_r_1 = min(high_s_r_1-1, low_s_r_1);
// setTrackbarPos("Low S","Object Detection_HSV_Red1", low_s_r_1);
// }
// void on_high_s_thresh_trackbar_red_1(int, void *){
// high_s_r_1 = max(high_s_r_1, low_s_r_1+1);
// setTrackbarPos("High S", "Object Detection_HSV_Red1", high_s_r_1);
// }
// void on_low_v_thresh_trackbar_red_1(int, void *){
// low_v_r_1= min(high_v_r_1-1, low_v_r_1);
// setTrackbarPos("Low V","Object Detection_HSV_Red1", low_v_r_1);
// }
// void on_high_v_thresh_trackbar_red_1(int, void *){
// high_v_r_1 = max(high_v_r_1, low_v_r_1+1);
// setTrackbarPos("High V", "Object Detection_HSV_Red1", high_v_r_1);
// }

// Trackbar for image threshodling in HSV colorspace : Red2