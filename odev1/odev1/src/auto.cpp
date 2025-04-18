#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include <algorithm>
#include <cmath>

ros::Publisher cmd_vel_pub;

float pitch_angle = 0.0;
float yaw_angle = 0.0;
float prev_pitch = 0.0;
float pitch_rate = 0.0;


float Kp_descending = 0.6;
float Kp_ascending = 0.85;

float current_linear = 0.0;
float current_angular = 0.0;
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    prev_pitch = pitch;

    pitch_angle = pitch;
    yaw_angle = yaw;
}

float smoothControl(float current_value, float target_value, float max_rate) {
    float difference = target_value - current_value;
    if (fabs(difference) > max_rate) {
        return current_value + (difference > 0 ? max_rate : -max_rate);
    }
    return target_value;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    geometry_msgs::Twist cmd;
    int size = msg->ranges.size();

    int front = size / 2;
    int left = size * 3 / 4;
    int right = size / 4;
    int front_left = size * 5 / 8;
    int front_right = size * 3 / 8;

    float front_dist = std::min(msg->ranges[front], 3.0f);
    float left_dist = std::min(msg->ranges[left], 3.0f);
    float right_dist = std::min(msg->ranges[right], 3.0f);
    float front_left_dist = std::min(msg->ranges[front_left], 3.0f);
    float front_right_dist = std::min(msg->ranges[front_right], 3.0f);

    ROS_WARN("Pitch: %.2f Yaw: %.2f Pitch Rate: %.2f Ön: %.2f FL: %.2f FR: %.2f L: %.2f R: %.2f", 
         pitch_angle, yaw_angle, pitch_rate, front_dist, front_left_dist, front_right_dist, left_dist, right_dist);

    float target_pitch = 0.0;
    float pitch_error = target_pitch - pitch_angle;
    
    float pitch_control_descending = Kp_descending * pitch_error;// + Kd_pitch * pitch_rate;
    float pitch_control_ascending = Kp_descending * pitch_error;// + Kd_pitch * pitch_rate;

    bool is_descending = pitch_angle > 0.001;
    bool is_ascending = pitch_angle < -0.001;
    bool is_flat = !is_descending && !is_ascending;

    float base_speed = 0.0;
    float turn_speed = 0.0;

    bool obstacle_front = front_dist > 0.2 && front_dist < 0.75;
    bool obstacle_front_left = front_left_dist > 0.2 && front_left_dist < 0.6;
    bool obstacle_front_right = front_right_dist > 0.2 && front_right_dist < 0.6;


    if (obstacle_front || obstacle_front_right || obstacle_front_left) {
        base_speed = 0.1;

        turn_speed = (front_right_dist > front_left_dist) ? -1.7 : 1.7;  
        ROS_WARN("Engel var!!");
    } else {
        if (is_descending) {
            base_speed = 0.5 + pitch_control_descending * 0.1; 
            turn_speed = 0.0;

            ROS_WARN("Rampa in → hiz: %.2f, donus: %.2f", base_speed, turn_speed);
        } 
        else if (is_ascending) {
            base_speed = 0.5 + pitch_control_ascending * 0.1;
            turn_speed = 0.0;
            ROS_WARN("Rampa cik");
        } 
        else {
            base_speed = 1.5;
            turn_speed = 0.0;
            ROS_WARN("Duz zemin git ");
        }
    }

    
    current_linear = smoothControl(current_linear, base_speed, 0.05);
    current_angular = smoothControl(current_angular, turn_speed, 0.25);  

    cmd.linear.x = current_linear;
    cmd.angular.z = current_angular;

    cmd_vel_pub.publish(cmd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "auto");
    ros::NodeHandle nh;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/p3at/cmd_vel", 10);
    ros::Subscriber laser_sub = nh.subscribe("/p3at/scan", 10, laserCallback);
    ros::Subscriber imu_sub = nh.subscribe("/p3at/imu_data", 10, imuCallback);

    
    ros::spin();
    return 0;
}
