#include "rvo_move_base_node.h"

uint64_t seq = 0;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "rvo_move_base_node");
    ros::NodeHandle n;
    rvo_node_pub = n.advertise<gazebo_msgs::WorldState>("rvo_vel", 1000);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    //ros::Subscriber sub = n.subscribe("/rvo/model_states", 100, rvo_velCallback);
    ros::Subscriber obstacle_sub = n.subscribe("/tracked_obstacle", 100, obstaclesCallback);
    ros::Subscriber amcl_pose_sub = n.subscribe("/amcl_pose", 100, amclPoseCallback);
    ros::Subscriber odom_sub = n.subscribe("/odom", 100, odomCallback);
    ros::ServiceServer service = n.advertiseService("set_rvo_goals", set_goals);
    ros::Rate loop_rate(50);

    if ((argc > 1) && (argc % 2 == 1))
    {
        int num_init_point = argc - 1;
        for (int i = 1; i < num_init_point + 1; i = i + 2)
        {
            geometry_msgs::Point point;
            point.x = atof(argv[i]);
            point.y = atof(argv[i + 1]);
            rvo_goals.push_back(point);
        }
    }
    else
    {
        ROS_INFO("No input, Using default position 0 1 0 2 ....0 10 ");
    }

    double neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed, goal_threshold;

    n.param<double>("neighborDist", neighborDist, 4);
    n.param<double>("maxNeighbors", maxNeighbors, 10);
    n.param<double>("timeHorizon", timeHorizon, 10);
    n.param<double>("timeHorizonObst", timeHorizonObst, 5);
    n.param<double>("radius", radius, 0.3);
    n.param<double>("maxSpeed", maxSpeed, 0.2);
    n.param<double>("goal_threshold", goal_threshold, 0.01);

    rvo = new RVO::RVOPlanner("gazebo");
    
    rvo->goal_threshold = goal_threshold;
    rvo->setupScenario(neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed);   // for exp
    // rvo->setupScenario(4.0f, 10, 10.0f, 5.0f, 0.3f, 0.2f); //for gazebo
    rvo_goals_init();

    std::cout<<"Configure completely"<<std::endl;

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool set_goals(rvo_ros::SetGoals::Request &req, rvo_ros::SetGoals::Response &res)
{
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

void rvo_velCallback(const gazebo_msgs::ModelStates::ConstPtr &sub_msg)
{
    // Process static obstacles
    rvo->setObstacles(sub_msg);

    //std::cout<<num_agent<<std::endl;
    seq++;
    int count_vel = 0;
    rvo->updateState_gazebo(sub_msg); // read the message
    if (motion_model == "default")
        rvo->setGoal(rvo_goals);
    else if (motion_model == "random")
        rvo->randGoal(limit_goal, "default");

    rvo->setInitial();
    rvo->setPreferredVelocities();

    arrived = rvo->arrived();

    std::vector<RVO::Vector2 *> new_velocities = rvo->step();

    auto models_name = sub_msg->name;
    int total_num = models_name.size();
    
    std_msgs::Header header;
    header.seq = seq;
    header.stamp = ros::Time::now();
    header.frame_id = "/world";

    msg_pub.header = header;
    msg_pub.name.clear();
    msg_pub.pose.clear();
    msg_pub.twist.clear();
    
    msg_pub.header = header;

    num_agent = new_velocities.size();

    if (num_agent != copy_num_agent)
    {
        std::cout << "The num of agents is " + std::to_string(num_agent) << std::endl;
        copy_num_agent = num_agent;
    }

    for (int i = 0; i < num_agent; i++)
    {
        geometry_msgs::Twist new_vel;
        geometry_msgs::Pose cur_pose;

        std::string agent_name = "agent" + std::to_string(i + 1);

        auto iter_agent = std::find(models_name.begin(), models_name.end(), agent_name);
        int iter_index = iter_agent - models_name.begin();

        
        if (iter_agent != models_name.end())
        {
            
            float x = new_velocities[count_vel]->x();
            float y = new_velocities[count_vel]->y();

            new_vel.linear.x = x;
            new_vel.linear.y = y;

            cur_pose = sub_msg->pose[iter_index];

            msg_pub.name.push_back(agent_name);
            msg_pub.twist.push_back(new_vel);
            msg_pub.pose.push_back(cur_pose);

            count_vel++;
            //std::cout << "Current " << agent_name << std::endl;
        }
    }
    rvo_node_pub.publish(msg_pub);
}


void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr amcl_pose)
{
    amcl_pose_ = *amcl_pose;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr odom)
{
    odom_ = *odom;
}

void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr new_obstacles)
{

    // Process static obstacles
    rvo->setObstacles(new_obstacles);

    //std::cout<<num_agent<<std::endl;
    seq++;
    int count_vel = 0;
    rvo->updateAgentStates(new_obstacles); // read the message
    rvo->updateRobotState(amcl_pose_, odom_);
    if (motion_model == "default")
        rvo->setGoal(rvo_goals);
    else if (motion_model == "random")
        rvo->randGoal(limit_goal, "default");

    rvo->setInitial();
    rvo->setPreferredVelocities();

    arrived = rvo->arrived();
    RVO::Vector2 new_velocities = rvo->getRobotCommand();
    
    geometry_msgs::Twist new_vel;
    new_vel.linear.x = new_velocities.x();
    new_vel.angular.z = new_velocities.y();
    new_vel = holonomicToNonholonomic(new_vel, amcl_pose_.pose.pose.orientation);
    cmd_vel_pub.publish(new_vel);
}

geometry_msgs::Twist holonomicToNonholonomic(geometry_msgs::Twist input, geometry_msgs::Quaternion orientation)
{
    geometry_msgs::Twist output;
    float angular_max = 1.0f;
    float angle_vel, angle_yaw;

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

        float diff = trans2pi(angle_yaw - angle_vel);

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
    output.linear.x = linear_x;
    output.angular.z = angular_z;
    return output;
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

