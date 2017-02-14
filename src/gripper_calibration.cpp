#include <lwr_gripper/gripper_calibration.hpp>

void callback(const sensor_msgs::JointState::ConstPtr state){
  current_effort_ = state->effort[0];
  current_position_ = state->position[0];
  filtered_val_.data = filtered_val_.data  *alpha_ + current_effort_* (1 - alpha_);
  pub_filtered_val_.publish(filtered_val_);
    
  if(std::abs(filtered_val_.data)>0.5){
    if(!forcing){
      forcing = true;
      begin = ros::Time::now();
    }else{
      ros::Duration time_spent = ros::Time::now() - begin;
      if(time_spent.toSec() > 0.1)
        gripper_calibrated_ = true;
    }
  }else
    forcing = false;  
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gripper_calibration");
  ros::NodeHandle nh;
  
  // TODO Param these !
  alpha_ = 0.95;
  filtered_val_.data = 0.0;
  current_cmd_.data = 1000.0;
  pub_cmd_ = nh.advertise<std_msgs::Float64>("/gripper/position_controller/command",1);
  pub_filtered_val_ = nh.advertise<std_msgs::Float64>("/gripper/filtered_effort",1);
  ros::Subscriber sub = nh.subscribe("/gripper/joint_states",1, callback);
  
  ros::Rate r(50);
  while(ros::ok() && !gripper_calibrated_){
    ros::spinOnce();
    r.sleep();
    pub_cmd_.publish(current_cmd_);
  }
  ROS_INFO("Gripper is calibrated");
  
  // Back off from the edges
  current_cmd_.data = current_position_- 30;
  pub_cmd_.publish(current_cmd_);
  
  // Save the zero calibration to a ros param
  ros::param::set("/gripper/position_zero",current_cmd_.data);
  
  ros::shutdown();
  return 0;
}