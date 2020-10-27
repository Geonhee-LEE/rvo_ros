#include "rvo_move_base_node.h"

uint64_t seq = 0;
bool goal_trigger_flg = false;
float control = 0.0;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "rvo_move_base_node");
    ros::NodeHandle n;
    rvo_node_pub = n.advertise<gazebo_msgs::WorldState>("rvo_vel", 1000);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber obstacle_sub = n.subscribe("/tracked_obstacle", 10, obstaclesCallback);
    ros::Subscriber amcl_pose_sub = n.subscribe("/amcl_pose", 10, amclPoseCallback);
    ros::Subscriber odom_sub = n.subscribe("/odom", 10, odomCallback);
    ros::ServiceServer service = n.advertiseService("set_rvo_goals", set_goals);
    ros::Rate loop_rate(10);

    double neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed, goal_threshold;

    n.param<double>("neighborDist", neighborDist, 4);
    n.param<double>("maxNeighbors", maxNeighbors, 10);
    n.param<double>("timeHorizon", timeHorizon, 10);
    n.param<double>("timeHorizonObst", timeHorizonObst, 5);
    n.param<double>("radius", radius, 0.3);
    n.param<double>("maxSpeed", maxSpeed, 0.2);
    n.param<double>("goal_threshold", goal_threshold, 0.01);

    rvo = new RVO::RVOPlanner("move_base");
    
    rvo->goal_threshold = goal_threshold;
    rvo->setupScenario(neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed);   // for exp
    // rvo->setupScenario(4.0f, 10, 10.0f, 5.0f, 0.3f, 0.2f); //for gazebo
    rvo_goals_init();

    std::cout<<"Configure completely"<<std::endl;

    while (ros::ok())
    {
        if(goal_trigger_flg)
        {
            rvo->setObstacles(new_obstacles);    // Process static obstacles 
            rvo->updateAgentStates(new_obstacles); // Process agents
            rvo->updateRobotState(amcl_pose_, odom_);  // Process target robot

            if (motion_model == "default")
                rvo->setRobotGoal(rvo_goals);
            else if (motion_model == "random")
                rvo->randGoal(limit_goal, "default");

            rvo->setInitial();
            rvo->setPreferredVelocities();

            if(rvo->robotArrived())
            {
                std::cout<<"Reached the Goal!!!" <<std::endl;
                goal_trigger_flg = false;
                geometry_msgs::Twist new_vel;
                cmd_vel_pub.publish(new_vel);
                ros::spinOnce();
                loop_rate.sleep();
            }

            RVO::Vector2 new_velocities = rvo->getRobotCommand();
            
            geometry_msgs::Twist new_vel;
            new_vel.linear.x = new_velocities.x();
            new_vel.linear.y = new_velocities.y();
            new_vel = holonomicToNonholonomic(new_vel, amcl_pose_.pose.pose.orientation);
            cmd_vel_pub.publish(new_vel);
            
        }
        ros::spinOnce();
        loop_rate.sleep();

    }
}

bool set_goals(rvo_ros::SetGoals::Request &req, rvo_ros::SetGoals::Response &res)
{
    goal_trigger_flg = true;
    
    if (req.model == "default")
    {
        motion_model = req.model;

        if (!rvo_goals.empty())
            rvo_goals.clear();

        for (const auto &coordinate : req.coordinates)
        {
            rvo_goals.push_back(coordinate);
        }

        res.num_goal = rvo_goals.size();

        return true;
    }

    if (req.model == "random")
    {
        motion_model = req.model;

        if (req.coordinates.size() < 2)
        {
            ROS_ERROR("too less input");
            return false;
        }
        else
        {
            limit_goal[0] = req.coordinates[0].x; // x_min
            limit_goal[1] = req.coordinates[1].x; // x_max
            limit_goal[2] = req.coordinates[0].y; // y_min
            limit_goal[3] = req.coordinates[1].y; // y_max

            res.num_goal = num_agent;
            rvo->randomOnceGoal(limit_goal);
            std::cout << "Current number of agent: " << num_agent << std::endl;
            return true;
        }
    }

    std::cout << "The specific model is wrong" << std::endl;
    return false;
}

void rvo_goals_init()
{

    if (rvo_goals.empty())
    {
        for (int i = 0; i < num_max; i++)
        {
            geometry_msgs::Point point;
            point.x = float(i);
            point.y = 1.0;
            rvo_goals.push_back(point);
        }
    }
}

void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr amcl_pose)
{
    amcl_pose_ = *amcl_pose;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr odom)
{
    odom_ = *odom;
}

void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr obstacles)
{    
    new_obstacles = *obstacles;
}

geometry_msgs::Twist holonomicToNonholonomic(geometry_msgs::Twist input, geometry_msgs::Quaternion orientation)
{
    geometry_msgs::Twist output;
    float angular_max = 0.30f;
    float angle_vel, angle_yaw, diff;

    float rvo_x = input.linear.x;
    float rvo_y = input.linear.y;

    float linear_x = 0.0;
    float angular_z = 0.0;

    if (rvo_x == 0.0f && rvo_y == 0.0f)
    {
        linear_x = 0;
        angular_z = 0;
    }
    else
    {
        double speed = sqrt(pow(rvo_x, 2) + pow(rvo_y, 2));

        if (rvo_y == 0)
            angle_vel = 0;
        else
            angle_vel = atan2(rvo_y, rvo_x);

        angle_yaw = cal_yaw(orientation);

        diff = trans2pi(angle_yaw - angle_vel);

        float speed_goal = speed * cos(diff);

        if (speed_goal < 0.01)
            linear_x = 0;
        else
            linear_x = speed_goal;   

        if (diff > 0.1)
            angular_z = diff < M_PI ? -angular_max : angular_max;
        else if (diff < -0.1)
            angular_z = diff < -M_PI ? -angular_max : angular_max;
        else
            angular_z = 0;
    }
    control = speed_smoothy(linear_x, control, 0.5f, 0.5f);
    output.linear.x = control;
    output.angular.z = angular_z;
    std::cout<<"diff: " << diff*180/3.14<< ", Non-holonimic velcity x: "<< output.linear.x<< ", angular z: " << output.angular.z <<std::endl;
    
    return output;
}

float speed_smoothy(float target, float control, float acc_inc, float acc_dec)
{
   if (target > control)
        control = std::min(target, control + acc_inc);
   else if (target < control) 
        control = std::max(target, control - acc_dec);
   else
        control = target;

    return control;

}

float cal_yaw(geometry_msgs::Quaternion quater)
{
  float x = quater.x;
  float y = quater.y;
  float z = quater.z;
  float w = quater.w;

  float raw = std::atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)));
  return raw;
}

float trans2pi(float angle)
{
  if (angle > M_PI)
    angle = angle - 2*M_PI;
  
  else if (angle < -M_PI)
    angle = angle + 2 * M_PI;

  return angle;

}

