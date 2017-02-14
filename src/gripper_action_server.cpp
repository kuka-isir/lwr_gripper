#include <actionlib/server/simple_action_server.h>
#include <lwr_gripper/GripperAction.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

class GripperActionServer{
  public:
    GripperActionServer(ros::NodeHandle& nh) :
    nh_(nh),
    act_srv_(nh, "gripper", boost::bind(&GripperActionServer::executeCB, this, _1), false)
    {
      act_srv_.start();
      pub_cmd_ = nh_.advertise<std_msgs::Float64>("/gripper/position_controller/command", 1);
      pub_filter_val_ = nh_.advertise<std_msgs::Float64>("/gripper/filtered_effort", 1);
      nh.getParam("/gripper/position_zero",position_zero_);
      ROS_INFO_STREAM("gripper zero : "<<position_zero_);
      opened_position = 0.0 + position_zero_;
      closed_position = -200.0 + position_zero_;
      ROS_INFO("gripper action server ready !");
      
      alpha_ = 0.95;
    }
    
  protected:
    ros::NodeHandle nh_;
    ros::Publisher pub_cmd_, pub_filter_val_;
    double current_effort_, filtered_val_, alpha_, current_position_, opened_position, closed_position, position_zero_;

    std_msgs::Float64 goal_cmd_;
    
    actionlib::SimpleActionServer<lwr_gripper::GripperAction> act_srv_;
    lwr_gripper::GripperFeedback feedback_;
    lwr_gripper::GripperResult result_;
    
    void executeCB(const lwr_gripper::GripperGoalConstPtr& goal){
      ros::Subscriber effort_sub_ = nh_.subscribe("/gripper/joint_states", 1, &GripperActionServer::effortCB,this);
      filtered_val_ = 0.0;
      
      bool finished_action = false, forcing = false;
      ros::Time begin;
      
      // Chose between opening and closing
      if(!goal->close)      
        goal_cmd_.data = opened_position;
      else
        goal_cmd_.data = closed_position;

      // Publish command
      pub_cmd_.publish(goal_cmd_);
      
      // Wait for the gripper to open a little because if closed it will be forcing already
      if(!goal->close)
        usleep(1e06);
      
      ros::Rate r(50);
      // Wait for position or max effort to be reached
      while(!finished_action){
        ros::spinOnce();
        r.sleep();
        if(std::abs(filtered_val_)>0.5){
          // if max effort start timer
          if(!forcing){
            forcing = true;
            begin = ros::Time::now();
            continue;
          }
          // if forcing time is too high consider goal reached
          ros::Duration time_spent = ros::Time::now() - begin;
          if(time_spent.toSec() > 0.1)
            finished_action = true;
        }
        else{
          forcing = false;
        }
        // if goal position is reached stop
        double min_val = goal_cmd_.data - 1;
        double max_val = goal_cmd_.data + 1;
        if((current_position_ >min_val) && (current_position_ <max_val))
          finished_action = true;
      }
      
      // publish command to current position to stop forcing
      std_msgs::Float64 current_cmd;
      current_cmd.data = current_position_;
      pub_cmd_.publish(current_cmd);
      
      // Action is finished
      result_.success = true;
      act_srv_.setSucceeded(result_);
      return;
    }
    
    void effortCB(const sensor_msgs::JointState::ConstPtr& msg){
      // save current position and effort
      current_position_ = msg->position[0];
      current_effort_ = msg->effort[0];
      
      // filter the efforts and published it
      filtered_val_ = filtered_val_*alpha_ + current_effort_*(1-alpha_);
      std_msgs::Float64 filter;
      filter.data = filtered_val_;
      pub_filter_val_.publish(filter);
    }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "gripper_control_action_server");
  
  ros::NodeHandle nh;
  GripperActionServer screwdriver_as(nh);
  
  while(ros::ok())
    ros::spinOnce();
  
  ros::shutdown();
  return 0;
}