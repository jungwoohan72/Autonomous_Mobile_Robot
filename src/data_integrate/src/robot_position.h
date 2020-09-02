#ifndef ROBOT_POSITION
#define ROBOT_POSITION

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

#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"
#include "core_msgs/robot_position.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <vector>


ros::Publisher pub_robot_position;
ros::Publisher pub_obstacle_position;
ros::Publisher pub_blue_position_absolute;

struct Point {
    float x;
    float y;
};

struct Line {
    float angle;
    Point point;
};

Point POINT_ROBOT_POSITION;
float ROBOT_RADIAN;

std::vector<Point> OBSTACLES {{4.0f, 1.5f}, {5.5f, 2.25f}, {7.0f, 1.5f}, {5.5f, 0.75f}};
std::vector<Point> BLUE_BALL_POSITION;

void robot_position_init(ros::NodeHandle nh);
void robot_position_update(void);
void obstacle_position_init(ros::NodeHandle nh);
void obstacle_position_update(void);
void blue_position_init(ros::NodeHandle nh);
void blue_position_update(void);
Point get_absolute_coordinate(const Point target, const Point robot_position, const float robot_orientation);
float get_robot_x(void);
float get_robot_y(void);
float get_robot_deg(void);
std::vector<float> square_error_to_wall(std::vector<Point> data, Point start, Point end);
std::vector<float> square_error_to_circle(std::vector<Point> data, Point center, float radius);
std::vector<float> square_error_to_arc_above(std::vector<Point> data);
std::vector<float> square_error_to_arc_below(std::vector<Point> data);
float average_square_error_to_map(std::vector<Point> data, float robot_x, float robot_y, float robot_deg);
float get_robot_end_x();
float get_robot_end_y();
#endif