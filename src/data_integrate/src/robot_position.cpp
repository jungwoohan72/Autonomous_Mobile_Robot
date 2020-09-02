#include "data_integration.h"
#include "robot_position.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)*M_PI/180.)
#define SQUARE(x) ((x)*(x))

Point POSITION_DELTA {0.0f, 0.0f};
float ANGLE_DELTA = 0.0f;

/*
* Make a topic "/robot_position",
* Ready to receive data from Lidar
*/
void robot_position_init(ros::NodeHandle nh) {
    //lidar_size; //(likely) 360 samples
    //lidar_degree.at(i) = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    //lidar_distance.at(i) = scan->ranges.at(i);
    pub_robot_position = nh.advertise<core_msgs::robot_position>("/robot_position", 1);
    POINT_ROBOT_POSITION = {2.8f, 0.5f};
    ROBOT_RADIAN = 3.14f;

    obstacle_position_init(nh);
    blue_position_init(nh);
}

/*
* Update the current position of the robot, and advertise.
* To be called when there's Lidar sensor update
*/
void robot_position_update() {
    //Create data
    std::vector<Point> data;
    for(int i = 0; i < lidar_size; i += 10) {
        if(std::isinf(lidar_distance.at(i))) continue;
        Point p;
        p.x = lidar_distance.at(i) * std::cos(DEG2RAD(lidar_degree.at(i)));
        p.y = lidar_distance.at(i) * std::sin(DEG2RAD(lidar_degree.at(i)));
        data.push_back(p);
    }
    //Generate particles
    std::vector<Point> possible_positions;
    std::vector<float> possible_orientations;

    std::vector<float> position_front_diff;
    std::vector<float> position_side_diff;
    std::vector<float> orientation_diff;

    //POSITION_DELTA.x *= 0.8f;
    //POSITION_DELTA.y *= 0.8f;
    if(ANGLE_DELTA > -60.0f && ANGLE_DELTA < 240.0f)
        ANGLE_DELTA = 0.0f;
    //ANGLE_DELTA *= 0.8f;
    position_front_diff.push_back(POSITION_DELTA.x);
    position_side_diff.push_back(POSITION_DELTA.y);
    orientation_diff.push_back(ANGLE_DELTA);
    for(int i = 1; i < 6; i++) {
        position_front_diff.push_back(POSITION_DELTA.x+i * 0.05f);
        position_front_diff.push_back(POSITION_DELTA.x+i * (-0.05f));
        position_side_diff.push_back(POSITION_DELTA.y+i * 0.05f);
        position_side_diff.push_back(POSITION_DELTA.y+i * (-0.05f));
        orientation_diff.push_back(ANGLE_DELTA+i * 5.0f);
        orientation_diff.push_back(ANGLE_DELTA+i * (-5.0f));
    }
    orientation_diff.push_back(-110.0f);
    orientation_diff.push_back(110.0f);

    //Calculate square error
    std::vector<float> minimum_square = {1e+38f, 0.0f, 0.0f, 0.0f};
    std::vector<float> second_minimum_square = {1.1e+38f, 0.0f, 0.0f, 0.0f};
    for(float front_d: position_front_diff) {
        for(float side_d: position_side_diff) {
            for(float angle_d: orientation_diff) {
                float new_deg = get_robot_deg() + angle_d;
                float new_x = get_robot_x() + front_d * std::cos(DEG2RAD(new_deg)) - side_d * std::sin(DEG2RAD(new_deg));
                float new_y = get_robot_y() + front_d * std::sin(DEG2RAD(new_deg)) + side_d * std::cos(DEG2RAD(new_deg));
                if(new_x < 0.0f || new_x > 8.0f || new_y < 0.0f || new_y > 3.0f) {
                    continue;
                }
                float square_error = average_square_error_to_map(data, new_x, new_y, new_deg);

                if(square_error < minimum_square.at(0)) {
                    second_minimum_square.at(0) = minimum_square.at(0);
                    second_minimum_square.at(1) = minimum_square.at(1);
                    second_minimum_square.at(2) = minimum_square.at(2);
                    second_minimum_square.at(3) = minimum_square.at(3);
                    minimum_square.at(0) = square_error;
                    minimum_square.at(1) = front_d;
                    minimum_square.at(2) = side_d;
                    minimum_square.at(3) = angle_d;
                }
                else if(square_error < second_minimum_square.at(0)) {
                    second_minimum_square.at(0) = square_error;
                    second_minimum_square.at(1) = front_d;
                    second_minimum_square.at(2) = side_d;
                    second_minimum_square.at(3) = angle_d;
                }
            }
        }
    }
    //Iteration 2
    float front_d_1 = 0.8f * minimum_square.at(1) + 0.2f * second_minimum_square.at(1);
    float side_d_1 = 0.8f * minimum_square.at(2) + 0.2f * second_minimum_square.at(2);
    float angle_d_1 = 0.8f * minimum_square.at(3) + 0.2f * second_minimum_square.at(3);
    position_front_diff.clear();
    position_side_diff.clear();
    orientation_diff.clear();
    for(int i = 1; i < 6; i++) {
        position_front_diff.push_back(front_d_1 + (float)(0.007 * i));
        position_front_diff.push_back(front_d_1 + (float)(0.007 * (-i)));
        position_side_diff.push_back(side_d_1 + (float)(0.007 * i));
        position_side_diff.push_back(side_d_1 + (float)(0.007 * (-i)));
        orientation_diff.push_back(angle_d_1 + (float)(0.7 * i));
        orientation_diff.push_back(angle_d_1 + (float)(0.7 * (-i)));
    }

    for(float front_d: position_front_diff) {
        for(float side_d: position_side_diff) {
            for(float angle_d: orientation_diff) {
                float new_deg = get_robot_deg() + angle_d;
                float new_x = get_robot_x() + front_d * std::cos(DEG2RAD(new_deg)) - side_d * std::sin(DEG2RAD(new_deg));
                float new_y = get_robot_y() + front_d * std::sin(DEG2RAD(new_deg)) + side_d * std::cos(DEG2RAD(new_deg));
                if(new_x < 0.0f || new_x > 8.0f || new_y < 0.0f || new_y > 3.0f) {
                    continue;
                }
                float square_error = average_square_error_to_map(data, new_x, new_y, new_deg);

                if(square_error < minimum_square.at(0)) {
                    second_minimum_square.at(0) = minimum_square.at(0);
                    second_minimum_square.at(1) = minimum_square.at(1);
                    second_minimum_square.at(2) = minimum_square.at(2);
                    second_minimum_square.at(3) = minimum_square.at(3);
                    minimum_square.at(0) = square_error;
                    minimum_square.at(1) = front_d;
                    minimum_square.at(2) = side_d;
                    minimum_square.at(3) = angle_d;
                }
                else if(square_error < second_minimum_square.at(0)) {
                    second_minimum_square.at(0) = square_error;
                    second_minimum_square.at(1) = front_d;
                    second_minimum_square.at(2) = side_d;
                    second_minimum_square.at(3) = angle_d;
                }
            }
        }
    }

    //Average out the result
    float alpha = 0.9f;
    float new_front_d = alpha * minimum_square.at(1) + (1.0f - alpha) * second_minimum_square.at(1);
    float new_side_d = alpha * minimum_square.at(2) + (1.0f - alpha) * second_minimum_square.at(2);
    float new_angle_d = alpha * minimum_square.at(3) + (1.0f - alpha) * second_minimum_square.at(3);
    //std::cout << "forward: " << new_front_d << " side: " << new_side_d << " anlge: " << new_angle_d << std::endl;
    //Update data
    ANGLE_DELTA = new_angle_d;
    ROBOT_RADIAN = DEG2RAD(get_robot_deg() + new_angle_d);
    if(ROBOT_RADIAN > 2*M_PI) ROBOT_RADIAN -= 2*M_PI;
    if(ROBOT_RADIAN < 0) ROBOT_RADIAN += 2*M_PI;
    POSITION_DELTA.x = new_front_d;
    POSITION_DELTA.y = new_side_d;
    POINT_ROBOT_POSITION.x = get_robot_x() + new_front_d * std::cos(DEG2RAD(get_robot_deg())) - new_side_d * std::sin(DEG2RAD(get_robot_deg()));
    POINT_ROBOT_POSITION.y = get_robot_y() + new_front_d * std::sin(DEG2RAD(get_robot_deg())) + new_side_d * std::cos(DEG2RAD(get_robot_deg()));

    //Advertise to the topic
    core_msgs::robot_position msg_robot_position;
    msg_robot_position.robot_x = get_robot_x();
    msg_robot_position.robot_y = get_robot_y();
    msg_robot_position.robot_deg = get_robot_deg();

    pub_robot_position.publish(msg_robot_position);
}

std::vector<float> square_error_to_wall(std::vector<Point> data, Point start, Point end) {
    std::vector<float> result;
    if(start.x == end.x) { //Vertical
        if(start.y > end.y) {
            Point temp;
            temp = start; start = end; end = temp;
        }
        for(Point p: data) {
            if(p.y < start.y) {
                result.push_back(SQUARE(p.x - start.x) + SQUARE(start.y - p.y));
            }
            else if(p.y > end.y) {
                result.push_back(SQUARE(p.x - end.x) + SQUARE(p.y - end.y));
            }
            else {
                result.push_back(SQUARE(p.x - start.x));
            }
        }
    }
    else { //Horizontal
        if(start.x > end.x) {
            Point temp;
            temp = start; start = end; end = temp;
        }
        for(Point p: data) {
            if(p.x < start.x) {
                result.push_back(SQUARE(start.x - p.x) + SQUARE(start.y - p.y));
            }
            else if(p.x > end.x) {
                result.push_back(SQUARE(p.x - end.x) + SQUARE(p.y - end.y));
            }
            else {
                result.push_back(SQUARE(p.y - start.y));
            }
        }
    }
    return result;
}

std::vector<float> square_error_to_circle(std::vector<Point> data, Point center, float radius) {
    std::vector<float> result;
    for(Point p: data) {
        float dist = std::hypotf(p.x - center.x, p.y - center.y);
        result.push_back(SQUARE(dist - radius));
    }
    return result;
}

std::vector<float> square_error_to_arc_above(std::vector<Point> data) {
    Point c1 = {2.0f, 2.0f};
    std::vector<float> result = square_error_to_circle(data, c1, 1.0f);
    for(int i = 0; i < data.size(); i++) {
        if(data.at(i).y < 2.0f) {
            float dist_to_right = hypotf(data.at(i).x - 3.0f, data.at(i).y - 2.0f);
            float dist_to_left = hypotf(data.at(i).x - 1.0f, data.at(i).y - 2.0f);
            result.at(i) = SQUARE(std::min(dist_to_left, dist_to_right));
        }
    }
    return result;
}

std::vector<float> square_error_to_arc_below(std::vector<Point> data) {
    Point c1 = {3.0f, 1.0f};
    std::vector<float> result = square_error_to_circle(data, c1, 1.0f);
    for(int i = 0; i < data.size(); i++) {
        if(data.at(i).y > 1.0f || data.at(i).x > 3.0f) {
            float dist_to_right = hypotf(data.at(i).x - 3.0f, data.at(i).y - 0.0f);
            float dist_to_left = hypotf(data.at(i).x - 2.0f, data.at(i).y - 1.0f);
            result.at(i) = SQUARE(std::min(dist_to_left, dist_to_right));
        }
    }
    return result;
}

float average_square_error_to_map(std::vector<Point> data, float robot_x, float robot_y, float robot_deg) {
    std::vector<Point> inverse_transformed_data;
    for(Point p: data) {
        Point temp;
        temp.x = p.x * std::cos(DEG2RAD(robot_deg)) - p.y * std::sin(DEG2RAD(robot_deg));
        temp.y = p.x * std::sin(DEG2RAD(robot_deg)) + p.y * std::cos(DEG2RAD(robot_deg));
        temp.x = temp.x + robot_x;
        temp.y = temp.y + robot_y;
        inverse_transformed_data.push_back(temp);
    }

    /*
        p-2-----------------p3  3
    p-1 p2                  p4
    |   |                       1.5
    p-3 p1                  p5
        p0------------------p6  0
1   2   3   4   5   6   7   8
    */
    Point p_minus1 = {2.0f, 2.0f};
    Point p0 = {3.0f, 0.0f};
    Point p1 = {3.0f, 1.0f};
    Point p2 = {3.0f, 2.0f};
    Point p_minus2 = {3.0f, 3.0f};
    Point p3 = {8.0f, 3.0f};
    Point p4 = {8.0f, 1.625f};
    Point p5 = {8.0f, 1.375f};
    Point p6 = {8.0f, 0.0f};
    Point p_minus3 = {2.0f, 1.0f};
    //Starting wall
    std::vector<float> square_wall_w = square_error_to_wall(inverse_transformed_data, p1, p_minus2);
    //Up wall
    std::vector<float> square_wall_n = square_error_to_wall(inverse_transformed_data, p_minus2, p3);
    //Goal wall
    std::vector<float> square_wall_e_1 = square_error_to_wall(inverse_transformed_data, p4, p3);
    std::vector<float> square_wall_e_2 = square_error_to_wall(inverse_transformed_data, p6, p5);
    //Down wall
    std::vector<float> square_wall_s = square_error_to_wall(inverse_transformed_data, p0, p6);

    //Wall behind the harvesting zone
    std::vector<float> square_wall_behind = square_error_to_wall(inverse_transformed_data, p_minus3, p_minus1);

    //Arc above
    std::vector<float> square_wall_arc_above = square_error_to_arc_above(inverse_transformed_data);
    //Arc below
    std::vector<float> square_wall_arc_below = square_error_to_arc_below(inverse_transformed_data);

    Point p7 = {4.0f, 1.5f};
    Point p8 = {5.5f, 2.25f};
    Point p9 = {7.0f, 1.5f};
    Point p10 = {5.5f, 0.75f};
    //Pole 0
    std::vector<float> square_wall_p1 = square_error_to_circle(inverse_transformed_data, p7, 0.07f);
    //Pole 1
    std::vector<float> square_wall_p2 = square_error_to_circle(inverse_transformed_data, p10, 0.07f);
    //Pole 2
    std::vector<float> square_wall_p3 = square_error_to_circle(inverse_transformed_data, p8, 0.07f);
    //Pole 3
    std::vector<float> square_wall_p4 = square_error_to_circle(inverse_transformed_data, p9, 0.07f);

    //Find minimum and summation
    float sum = 0.0f;
    for(int i = 0; i < inverse_transformed_data.size(); i++) {
        float min = square_wall_w.at(i);
        if(min > square_wall_n.at(i)) min = square_wall_n.at(i);
        if(min > square_wall_e_1.at(i)) min = square_wall_e_1.at(i);
        if(min > square_wall_e_2.at(i)) min = square_wall_e_2.at(i);
        if(min > square_wall_s.at(i)) min = square_wall_s.at(i);
        if(min > square_wall_behind.at(i)) min = square_wall_behind.at(i);
        if(min > square_wall_arc_above.at(i)) min = square_wall_arc_above.at(i);
        if(min > square_wall_arc_below.at(i)) min = square_wall_arc_below.at(i);
        if(min > square_wall_p1.at(i)) min = square_wall_p1.at(i);
        if(min > square_wall_p2.at(i)) min = square_wall_p2.at(i);
        if(min > square_wall_p3.at(i)) min = square_wall_p3.at(i);
        if(min > square_wall_p4.at(i)) min = square_wall_p4.at(i);
        sum += SQUARE(min) / (float)(inverse_transformed_data.size());
    }
    return sum;
}

float visible_angle = 45.0f;

void obstacle_position_init(ros::NodeHandle nh) {
    pub_obstacle_position = nh.advertise<core_msgs::ball_position>("/obstacle_position", 3);
}

void obstacle_position_update() {
    std::vector<Point> new_obstacles {{4.0f, 1.5f}, {5.5f, 2.25f}, {7.0f, 1.5f}, {5.5f, 0.75f}};

    //Add red balls based on camera inputs
    for(int i = 0; i < detected_red_ball_num; i++) {
        float x = red_ball_Y.at(i) + 0.195f;
        float y = -1 * red_ball_X.at(i);
        float ratio = std::hypotf(x, y) / std::abs(x);
        Point obs;
        obs.x = ratio * x; obs.y = ratio * y;
        Point p = get_absolute_coordinate(obs, POINT_ROBOT_POSITION, get_robot_deg());
        if(p.x < 0.0f || p.x > 8.0f || p.y < 0.0f || p.y > 3.0f) {
            continue;
        }
        float relative_angle;
        relative_angle = RAD2DEG(std::atan2(obs.y - get_robot_y(), obs.x - get_robot_x()));
        float angle_diff = relative_angle - get_robot_deg();
        angle_diff = std::remainderf(angle_diff, 360.0f);
        if(angle_diff > visible_angle && angle_diff < 360.0f - visible_angle) { //not reliable
            continue;
        }
        new_obstacles.push_back(p);
    }
    for(int i = 0; i < detected_rear_up_red_ball_num; i++) {
        float x = -rear_up_red_ball_Y.at(i) - 0.2f;
        float y = -1 * (-rear_up_red_ball_X.at(i));
        float ratio = std::hypotf(x, y) / std::abs(x);
        Point obs;
        obs.x = ratio * x; obs.y = ratio * y;
        Point p = get_absolute_coordinate(obs, POINT_ROBOT_POSITION, get_robot_deg());
        if(p.x < 0.0f || p.x > 8.0f || p.y < 0.0f || p.y > 3.0f) {
            continue;
        }
        float relative_angle;
        relative_angle = RAD2DEG(std::atan2(obs.y - get_robot_y(), obs.x - get_robot_x()));
        float angle_diff = relative_angle - get_robot_deg();
        angle_diff = std::remainderf(angle_diff, 360.0f);
        if(angle_diff > 180.0f + visible_angle || angle_diff < 180.0f - visible_angle) { //not reliable
            continue;
        }
        new_obstacles.push_back(p);
    }
    /*
    If the old data should be visible but not found, it's deprecated.
    If the old data has the close data from the new data, merge them into a new data
    If the old data has no close data from the new data, and is not visible, keep that data.
    Finally, merge if the data are close enough to each other.
    */
    /*
    Create mapping from the old data to new data
    Found: Index
    Not found and in sight
    Not found and out of sight
    */
    //Remove the balls should be visible, but are not
    for(int i = 0; i < OBSTACLES.size(); i++) {
        Point old_obstacle = OBSTACLES.at(i);
        if(std::hypotf(old_obstacle.x - get_robot_x(), old_obstacle.y - get_robot_y()) < 0.60f) {
            continue;
        }
        float relative_angle;
        relative_angle = RAD2DEG(std::atan2(old_obstacle.y - get_robot_y(), old_obstacle.x - get_robot_x()));
        float angle_diff = relative_angle - get_robot_deg();
        angle_diff = std::remainderf(angle_diff, 360.0f);
        if(angle_diff < (visible_angle - 2.0f) || angle_diff > 360.0f - (visible_angle - 2.0f)) { //Visible from upper camera
            OBSTACLES.erase(OBSTACLES.begin() + i);
        }
        if(angle_diff > 180.0f - (visible_angle - 2.0f) && angle_diff < 180.0f - (visible_angle - 2.0f)) { //Visible from rear camera
            OBSTACLES.erase(OBSTACLES.begin() + i);
        }
    }

    //Merge close obstacles from the previous data to one
    std::vector<Point> filtered_obstacles;
    for(Point new_obstacle: new_obstacles) {
        bool existing = false;
        for(Point old_obstacle: OBSTACLES) {
            float dist = std::hypotf(new_obstacle.x - old_obstacle.x, new_obstacle.y - old_obstacle.y);
            if(dist < 0.30f) {
                Point filtered_position;
                float alpha = 0.5f;
                filtered_position.x = alpha * new_obstacle.x + (1 - alpha) * old_obstacle.x;
                filtered_position.y = alpha * new_obstacle.y + (1 - alpha) * old_obstacle.y;
                filtered_obstacles.push_back(filtered_position);
                existing = true;
                break;
            }
        }
        if(!existing) {
            filtered_obstacles.push_back(new_obstacle);
        }
    }

    //Add remaining obstacles from the previous data
    for(Point old_obstacle: OBSTACLES) {
        bool pushed = false;
        for(Point filtered_obstacle: filtered_obstacles) {
            float dist = std::hypotf(filtered_obstacle.x - old_obstacle.x, filtered_obstacle.y - old_obstacle.y);
            if(dist < 0.30f) {
                pushed = true;
                break;
            }
        }
        if(!pushed) {
            filtered_obstacles.push_back(old_obstacle);
        }
    }

    OBSTACLES.assign(filtered_obstacles.begin(), filtered_obstacles.end());

    core_msgs::ball_position msg_obstacle_position;
    msg_obstacle_position.size = OBSTACLES.size();
    for(Point obs: OBSTACLES) {
        msg_obstacle_position.img_x.push_back(obs.x);
        msg_obstacle_position.img_y.push_back(obs.y);
    }

    pub_obstacle_position.publish(msg_obstacle_position);
}

void blue_position_init(ros::NodeHandle nh) {
    pub_blue_position_absolute = nh.advertise<core_msgs::ball_position>("/absolute_position_blue", 3);
}

void blue_position_update() {
    std::vector<Point> new_blues;

    //Add blue balls based on camera inputs
    for(int i = 0; i < detected_blue_ball_num; i++) {
        float x = blue_ball_Y.at(i) + 0.195f;
        float y = -1 * blue_ball_X.at(i);
        float ratio = std::hypotf(x, y) / std::abs(x);
        Point ball;
        ball.x = ratio * x; ball.y = ratio * y;
        Point p = get_absolute_coordinate(ball, POINT_ROBOT_POSITION, get_robot_deg());
        if(p.x < 0.0f || p.x > 8.0f || p.y < 0.0f || p.y > 3.0f) {
            continue;
        }
        float relative_angle;
        relative_angle = RAD2DEG(std::atan2(p.y - get_robot_y(), p.x - get_robot_x()));
        float angle_diff = relative_angle - get_robot_deg();
        angle_diff = std::remainderf(angle_diff, 360.0f);
        if(angle_diff > visible_angle && angle_diff < 360.0f - visible_angle) { //Not reliable
            continue;
        }
        new_blues.push_back(p);
    }
    for(int i = 0; i < detected_rear_up_blue_ball_num; i++) {
        float x = -rear_up_blue_ball_Y.at(i) - 0.2f;
        float y = -1 * (-rear_up_blue_ball_X.at(i));
        float ratio = std::hypotf(x, y) / std::abs(x);
        Point ball;
        ball.x = ratio * x; ball.y = ratio * y;
        Point p = get_absolute_coordinate(ball, POINT_ROBOT_POSITION, get_robot_deg());
        if(p.x < 0.0f || p.x > 8.0f || p.y < 0.0f || p.y > 3.0f) {
            continue;
        }
        float relative_angle;
        relative_angle = RAD2DEG(std::atan2(p.y - get_robot_y(), p.x - get_robot_x()));
        float angle_diff = relative_angle - get_robot_deg();
        angle_diff = std::remainderf(angle_diff, 360.0f);
        if(angle_diff > 180.0f + visible_angle || angle_diff < 180.0f - visible_angle) { //Not reliable
            continue;
        }
        new_blues.push_back(p);
    }
    /*
    If the old data should be visible but not found, it's deprecated.
    If the old data has the close data from the new data, merge them into a new data
    If the old data has no close data from the new data, and is not visible, keep that data.
    Finally, merge if the data are close enough to each other.
    */
    /*
    Create mapping from the old data to new data
    Found: Index
    Not found and in sight
    Not found and out of sight
    */
    //Remove the balls should be visible, but are not
    for(int i = 0; i < BLUE_BALL_POSITION.size(); i++) {
        Point old_position = BLUE_BALL_POSITION.at(i);
        float relative_angle;
        relative_angle = RAD2DEG(std::atan2(old_position.y - get_robot_y(), old_position.x - get_robot_x()));
        float angle_diff = relative_angle - get_robot_deg();
        angle_diff = std::remainderf(angle_diff, 360.0f);
        if(angle_diff < (visible_angle - 2.0f) || angle_diff > 360.0f - (visible_angle - 2.0f)) { //Visible from upper camera
            BLUE_BALL_POSITION.erase(BLUE_BALL_POSITION.begin() + i);
        }
        if(angle_diff > 180.0f - (visible_angle - 2.0f) && angle_diff < 180.0f - (visible_angle - 2.0f)) { //Visible from rear camera
            BLUE_BALL_POSITION.erase(BLUE_BALL_POSITION.begin() + i);
        }
    }

    //Merge close blue balls from the previous data to one
    std::vector<Point> filtered_blues;
    for(Point new_position: new_blues) {
        bool existing = false;
        for(Point old_position: BLUE_BALL_POSITION) {
            float dist = std::hypotf(new_position.x - old_position.x, new_position.y - old_position.y);
            if(dist < 0.30f) {
                Point filtered_position;
                float alpha = 0.5f;
                filtered_position.x = alpha * new_position.x + (1 - alpha) * old_position.x;
                filtered_position.y = alpha * new_position.y + (1 - alpha) * old_position.y;
                filtered_blues.push_back(filtered_position);
                existing = true;
                break;
            }
        }
        if(!existing) {
            filtered_blues.push_back(new_position);
        }
    }

    //Add remaining blue balls from the previous data
    for(Point old_position: BLUE_BALL_POSITION) {
        bool pushed = false;
        for(Point filtered_position: filtered_blues) {
            float dist = std::hypotf(filtered_position.x - old_position.x, filtered_position.y - old_position.y);
            if(dist < 0.30f) {
                pushed = true;
                break;
            }
        }
        if(!pushed) {
            filtered_blues.push_back(old_position);
        }
    }

    BLUE_BALL_POSITION.assign(filtered_blues.begin(), filtered_blues.end());

    core_msgs::ball_position msg_blue_position_absolute;
    msg_blue_position_absolute.size = BLUE_BALL_POSITION.size();
    for(Point pos: BLUE_BALL_POSITION) {
        msg_blue_position_absolute.img_x.push_back(pos.x);
        msg_blue_position_absolute.img_y.push_back(pos.y);
    }

    pub_blue_position_absolute.publish(msg_blue_position_absolute);
}

Point get_absolute_coordinate(const Point target, const Point robot_position, const float robot_orientation) {
    Point result;
    result.x = robot_position.x + target.x * std::cos(DEG2RAD(robot_orientation)) - target.y * std::sin(DEG2RAD(robot_orientation));
    result.y = robot_position.y + target.x * std::sin(DEG2RAD(robot_orientation)) + target.y * std::cos(DEG2RAD(robot_orientation));
    return result;
}

float get_robot_x() {
    return POINT_ROBOT_POSITION.x;
}
float get_robot_y() {
    return POINT_ROBOT_POSITION.y;
}
float get_robot_deg() {
    return RAD2DEG(ROBOT_RADIAN);
}

float get_robot_end_x() {
    return POINT_ROBOT_POSITION.x + (0.34f)*std::cos(ROBOT_RADIAN);
}
float get_robot_end_y() {
    return POINT_ROBOT_POSITION.y + (0.34f)*std::sin(ROBOT_RADIAN);
}