#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "core_msgs/ball_position.h"
#include "core_msgs/robot_position.h"

#include "opencv2/opencv.hpp"

float MAP_CX = 200.5;
float MAP_CY = 200.5;
float MAP_RESOL = 0.015;             // Map resoultion [cm]
int MAP_WIDTH = 480;
int MAP_HEIGHT = 480;
int MAP_CENTER = 50;
int OBSTACLE_PADDING = 1;           // Obstacle Size
int OBSTACLE_CONNECT_MAX = 15;      // Range to connect obstacles

int init_ball;
int init_lidar;

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];

int blue_ball_num;
std::vector<float> blue_ball_X(20);
std::vector<float> blue_ball_Y(20);

float robot_x;
float robot_y;
float robot_deg;

int obstacle_num;
std::vector<float> obstacle_X(20);
std::vector<float> obstacle_Y(20);

int route_num;
std::vector<float> route_X(20);
std::vector<float> route_Y(20);

boost::mutex map_mutex;


#define RAD2DEG(x) ((x)*180./M_PI)

bool check_point_range(int cx, int cy)
{
    return (cx<MAP_WIDTH-1)&&(cx>0)&&(cy<MAP_HEIGHT-1)&&(cy>0);
}

void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //map_mutex.lock();
    int count = scan->angle_max / scan->angle_increment;
    lidar_size=count;
    for(int i = 0; i < count; i++)
    {
        lidar_degree[i] = scan->angle_min + scan->angle_increment * i;
        lidar_distance[i]=scan->ranges[i];
    }
    //map_mutex.unlock();

}

void robot_position_Callback(const core_msgs::robot_position::ConstPtr& position) {
    robot_x = position -> robot_x;
    robot_y = position -> robot_y;
    robot_deg = position -> robot_deg;
    return;
}

void obstacle_position_Callback(const core_msgs::ball_position::ConstPtr& position_list) {
    obstacle_num = position_list -> size;
    obstacle_X.assign(position_list->img_x.begin(), position_list->img_x.end());
    obstacle_Y.assign(position_list->img_y.begin(), position_list->img_y.end());
    return;
}

void blue_ball_position_Callback(const core_msgs::ball_position::ConstPtr& position_list) {
    blue_ball_num = position_list -> size;
    blue_ball_X.assign(position_list->img_x.begin(), position_list->img_x.end());
    blue_ball_Y.assign(position_list->img_y.begin(), position_list->img_y.end());
    return;
}

void route_Callback(const core_msgs::ball_position::ConstPtr& position_list) {
    route_num = position_list -> size;
    route_X.assign(position_list->img_x.begin(), position_list->img_x.end());
    route_Y.assign(position_list->img_y.begin(), position_list->img_y.end());
    return;
}

inline int map_X(double x) {
    return MAP_WIDTH/2 + (int)((x - 5.5) / MAP_RESOL);
}
inline int map_Y(double y) {
    return MAP_HEIGHT/2 - (int)((y - 1.5) / MAP_RESOL);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_show_node");
    ros::NodeHandle n;

    ros::Subscriber sub1 = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, lidar_Callback);
    ros::Subscriber sub2 = n.subscribe<core_msgs::robot_position>("/robot_position", 1, robot_position_Callback);
    ros::Subscriber sub3 = n.subscribe<core_msgs::ball_position>("/obstacle_position", 1, obstacle_position_Callback);
    ros::Subscriber sub4 = n.subscribe<core_msgs::ball_position>("/absolute_position_blue", 1, blue_ball_position_Callback);
    ros::Subscriber sub5 = n.subscribe<core_msgs::ball_position>("/route", 1, route_Callback);

    ros::Rate loop_rate(50);
    while(ros::ok){
        ros::spinOnce();
        cv::Mat map = cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8UC3);
        // Drawing Map
        // (3,0) -> (8,0) -> (8,3) -> (3,3) -> (3,1)
        cv::line(map, cv::Point(map_X(3.0), map_Y(0.0)),cv::Point(map_X(8.0), map_Y(0.0)),cv::Scalar(255,255,255), 1);
        cv::line(map, cv::Point(map_X(8.0), map_Y(0.0)),cv::Point(map_X(8.0), map_Y(3.0)),cv::Scalar(255,255,255), 1);
        cv::line(map, cv::Point(map_X(8.0), map_Y(3.0)),cv::Point(map_X(3.0), map_Y(3.0)),cv::Scalar(255,255,255), 1);
        cv::line(map, cv::Point(map_X(3.0), map_Y(3.0)),cv::Point(map_X(3.0), map_Y(1.0)),cv::Scalar(255,255,255), 1);


        int cx, cy;
        int cx1, cx2, cy1, cy2;

        // Drawing obstacles
        for(int i = 0; i < obstacle_num; i++)
        {
            cx = map_X(obstacle_X[i]);
            cy = map_Y(obstacle_Y[i]);
            cx1 = cx-OBSTACLE_PADDING*2;
            cy1 = cy-OBSTACLE_PADDING*2;
            cx2 = cx+OBSTACLE_PADDING*2;
            cy2 = cy+OBSTACLE_PADDING*2;

            if(check_point_range(cx,cy) && check_point_range(cx1,cy1) && check_point_range(cx2,cy2))
            {
                cv::circle(map,cv::Point(cx, cy), (int)(0.33/MAP_RESOL), cv::Scalar(0,0,50), -1);
            }
        }

        for(int i = 0; i < obstacle_num; i++)
        {
            cx = map_X(obstacle_X[i]);
            cy = map_Y(obstacle_Y[i]);
            cx1 = cx-OBSTACLE_PADDING*2;
            cy1 = cy-OBSTACLE_PADDING*2;
            cx2 = cx+OBSTACLE_PADDING*2;
            cy2 = cy+OBSTACLE_PADDING*2;

            if(check_point_range(cx,cy) && check_point_range(cx1,cy1) && check_point_range(cx2,cy2))
            {
                cv::circle(map,cv::Point(cx, cy),(int)(0.07/MAP_RESOL), cv::Scalar(0,0,255), -1);
            }
        }
        // Drawing Lidar data
        float obstacle_x, obstacle_y;
        for(int i = 0; i < lidar_size; i++)
        {
            obstacle_x = robot_x+lidar_distance[i]*cos(lidar_degree[i] + robot_deg/180.0*3.14159);
            obstacle_y = robot_y+lidar_distance[i]*sin(lidar_degree[i] + robot_deg/180.0*3.14159);
            cx = map_X((double)obstacle_x);
            cy = map_Y((double)obstacle_y);
            cx1 = cx-OBSTACLE_PADDING;
            cy1 = cy-OBSTACLE_PADDING;
            cx2 = cx+OBSTACLE_PADDING;
            cy2 = cy+OBSTACLE_PADDING;

            if(check_point_range(cx,cy) && check_point_range(cx1,cy1) && check_point_range(cx2,cy2))
            {
                cv::rectangle(map,cv::Point(cx1, cy1),cv::Point(cx2, cy2), cv::Scalar(100,100,100), -1);
            }
        }

        // Drawing blue ball
        for(int i = 0; i < blue_ball_num; i++)
        {
            cx = map_X((double)blue_ball_X[i]);
            cy = map_Y((double)blue_ball_Y[i]);
            cx1 = cx-OBSTACLE_PADDING*2;
            cy1 = cy-OBSTACLE_PADDING*2;
            cx2 = cx+OBSTACLE_PADDING*2;
            cy2 = cy+OBSTACLE_PADDING*2;
            //std::cout << "drawing ball at x:" << cx << "y:" << cy << std::endl;

            if(check_point_range(cx,cy) && check_point_range(cx1,cy1) && check_point_range(cx2,cy2))
            {
                cv::rectangle(map,cv::Point(cx1, cy1),cv::Point(cx2, cy2), cv::Scalar(255,0,0), -1);
            }
        }

        // Drawing ROBOT
        cv::circle(map,cv::Point(map_X((double)robot_x), map_Y((double)robot_y)),3,cv::Scalar(0,255,0),-1);


        // Drawing route
        if(route_num > 0) {
            cv::line(map, cv::Point(map_X((double)robot_x), map_Y((double)robot_y)),
                    cv::Point(map_X((double)route_X.at(0)), map_Y((double)route_Y.at(0))),
                cv::Scalar(255,100,100), 1);
        }
        for(int i = 0; i < route_num - 1; i++) {
            cv::line(map, cv::Point(map_X((double)route_X.at(i)), map_Y((double)route_Y.at(i))),
                    cv::Point(map_X((double)route_X.at(i+1)), map_Y((double)route_Y.at(i+1))),
                    cv::Scalar(255,100,100), 1);
        }

        cv::imshow("Frame",map);

        if(cv::waitKey(10)==113){  //wait for a key command. if 'q' is pressed, then program will be terminated.
            return 0;
        }
        loop_rate.sleep();
    }



    return 0;
}
