#ifndef LWRGRIPPER_GRIPPERCALIBRATION_HPP
#define LWRGRIPPER_GRIPPERCALIBRATION_HPP

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

void callback(const sensor_msgs::JointState::ConstPtr state);

double alpha_, current_effort_;
bool gripper_calibrated_ = false;
std_msgs::Float64 filtered_val_, current_cmd_;

ros::Time begin;
bool forcing = false;

ros::Publisher pub_filtered_val_, pub_cmd_;

#endif // LWRGRIPPER_GRIPPERCALIBRATION_HPP