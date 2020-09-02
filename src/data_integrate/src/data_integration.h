#ifndef DATA_INTEGRATION
#define DATA_INTEGRATION
#include <vector>

int lidar_size;
std::vector<float> lidar_degree(400);
std::vector<float> lidar_distance(400);
float lidar_obs;

int detected_blue_ball_num;
std::vector<float> blue_ball_X(20);
std::vector<float> blue_ball_Y(20);

int detected_lower_blue_ball_num;
std::vector<float> lower_blue_ball_X(20);
std::vector<float> lower_blue_ball_Y(20);

int detected_rear_up_blue_ball_num;
std::vector<float> rear_up_blue_ball_X(20);
std::vector<float> rear_up_blue_ball_Y(20);

int detected_rear_blue_ball_num;
std::vector<float> rear_blue_ball_X(20);
std::vector<float> rear_blue_ball_Y(20);

int detected_red_ball_num;
std::vector<float> red_ball_X(20);
std::vector<float> red_ball_Y(20);

int detected_rear_up_red_ball_num;
std::vector<float> rear_up_red_ball_X(20);
std::vector<float> rear_up_red_ball_Y(20);

int detected_rear_red_ball_num;
std::vector<float> rear_red_ball_X(20);
std::vector<float> rear_red_ball_Y(20);

int detected_green_ball_num;
std::vector<float> green_ball_X(10);
std::vector<float> green_ball_Y(10);

int detected_green_ball_num_lower;
std::vector<float> green_ball_X_lower(10);
std::vector<float> green_ball_Y_lower(10);

int detected_rear_up_green_ball_num;
std::vector<float> rear_up_green_ball_X(10);
std::vector<float> rear_up_green_ball_Y(10);

int detected_green_ball_num_rear;
std::vector<float> green_ball_X_rear(10);
std::vector<float> green_ball_Y_rear(10);

#endif