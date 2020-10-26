#ifndef RVO_NODE_MOVE_BASE_H
#define RVO_NODE_MOVE_BASE_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Header.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/WorldState.h"
#include <string>
#include "../rvo_lib/move_base_rvo.h"
#include "rvo_ros/SetGoals.h"
#include "gazebo_msgs/WorldState.h"
#include "gazebo_msgs/ModelStates.h"
#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#define _USE_MATH_DEFINES
#include <math.h>

const int num_max = 30;
int num_agent = 0;
int copy_num_agent = 1;
bool arrived = false;
float vel_ratio(float vel, float lo, float hi);
ros::Publisher rvo_node_pub;
ros::Publisher cmd_vel_pub;
gazebo_msgs::WorldState msg_pub;
std::vector<geometry_msgs::Point> rvo_goals;

geometry_msgs::PoseWithCovarianceStamped amcl_pose_;
nav_msgs::Odometry odom_;

RVO::RVOPlanner* rvo;
std::string motion_model = "default";

void rvo_velCallback(const gazebo_msgs::ModelStates::ConstPtr& sub_msg);
void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr);
void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr);
void odomCallback(const nav_msgs::Odometry::ConstPtr);

bool set_goals(rvo_ros::SetGoals::Request &req, rvo_ros::SetGoals::Response &res);
geometry_msgs::Twist holonomicToNonholonomic(geometry_msgs::Twist input, geometry_msgs::Quaternion );
float cal_yaw(geometry_msgs::Quaternion quater);
float trans2pi(float angle);
void rvo_goals_init();
float limit_goal[4];

#endif