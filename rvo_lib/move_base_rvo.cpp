#include "move_base_rvo.h"

#include <utility>

namespace RVO
{

RVOPlanner::RVOPlanner(std::string simulation) : simulator(std::move(simulation))
{
    sim = new RVO::RVOSimulator();
};

void RVOPlanner::setupScenario(float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed)
{
    sim->setAgentDefaults(neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed);
}

// set the goal manually
// default: invert direction
// random: change the goal every step

void RVOPlanner::setGoal()
{
    for (size_t i = 0; i < sim->getNumAgents(); ++i)
    {
        if (absSq(goals[i] - sim->getAgentPosition(i)) < goal_threshold)
        {
            // goals[i] = -sim->getAgentPosition(i);
            goals[i] = -goals[i];
        }
    }
}

void RVOPlanner::setGoal(std::vector<geometry_msgs::Point> set_goals)
{
    goals.clear();
    int num_agent = sim->agents_.size();
    if (set_goals.size() < num_agent)
        std::cout << "error:The num of goals is less than agents" << std::endl;
    else
    {
        for (int i = 0; i < num_agent; i++)
        {
            float x = set_goals[i].x;
            float y = set_goals[i].y;

            goals.emplace_back(Vector2(x, y));
            //std::cout<<"goal"+ std::to_string(i+1) + ":["<<x<<","<<y<<"]"<<std::endl;
        }
    }
}

void RVOPlanner::setRobotGoal(std::vector<geometry_msgs::Point> set_goals)
{

    float x = set_goals[0].x;
    float y = set_goals[0].y;

    goals.emplace_back(Vector2(x, y));   
    
}

void RVOPlanner::randomOnceGoal(const float limit_goal[4])
{
    float x_min = limit_goal[0];
    float x_max = limit_goal[1];
    float y_min = limit_goal[2];
    float y_max = limit_goal[3];

    std::random_device rd;
    std::default_random_engine e(rd());
    std::uniform_real_distribution<float> ux(x_min, x_max);
    std::uniform_real_distribution<float> uy(y_min, y_max);
    std::uniform_int_distribution<int> ur(0, 10);

    for (size_t i = 0; i < sim->getNumAgents(); ++i)
    {

        float x = ux(e);
        float y = uy(e);
        int rand = ur(e);
    
        goals[i] = (Vector2(x, y));

        std::cout<< "random once successfully" <<std::endl;
    }
       
}

void RVOPlanner::randGoal(const float limit_goal[4], const std::string &model)
{

    float x_min = limit_goal[0];
    float x_max = limit_goal[1];
    float y_min = limit_goal[2];
    float y_max = limit_goal[3];

    std::random_device rd;
    std::default_random_engine e(rd());
    std::uniform_real_distribution<float> ux(x_min, x_max);
    std::uniform_real_distribution<float> uy(y_min, y_max);
    std::uniform_int_distribution<int> ur(0, 10);

    for (size_t i = 0; i < sim->getNumAgents(); ++i)
    {

        float x = ux(e);
        float y = uy(e);
        int rand = ur(e);

        if (!IfInitial)
            goals.emplace_back(x, y);

        else if (model == "default")
        {
            if (absSq(goals[i] - sim->getAgentPosition(i)) < goal_threshold)
                goals[i] = (Vector2(x, y));
        }
        else if (model == "random")
        {
            if (rand > 8)
                goals[i] = (Vector2(x, y));
        }
    }
}

bool RVOPlanner::arrived()
{
    bool reach = true;

    for (size_t i = 0; i < sim->getNumAgents(); ++i)
    {
        if (absSq(goals[i] - sim->getAgentPosition(i)) >= goal_threshold)
            reach = false;
    }

    return reach;
}

bool RVOPlanner::robotArrived()
{
    bool reach = true;

    size_t robot_id = sim->getNumAgents();

    if (absSq(goals[0] - sim->getAgentPosition(0)) >= goal_threshold)
    {
        reach = false;    
    }    

    return reach;
}
void RVOPlanner::setInitial()
{
    IfInitial = (!goals.empty()) && (!sim->agents_.empty());
}
void RVOPlanner::setObstacles(gazebo_msgs::ModelStates::ConstPtr model_msg)
{
    if (simulator == "gazebo")
    {
        auto models_name = model_msg->name;
        int num = models_name.size();
        int count = 0;
        std::string agent_name= "unit_box";

        sim->obstacles_.clear();

        for (int i = 0; i < num; i++)
        {
            auto iter_agent = std::find(models_name.begin(), models_name.end(), agent_name);
            int agent_index = iter_agent - models_name.begin();

            if (iter_agent != models_name.end())
            {
                // Add (polygonal) obstacle(s), specifying vertices in counterclockwise order.
                std::vector<RVO::Vector2> vertices;
                vertices.push_back(RVO::Vector2(model_msg->pose[agent_index].position.x - 0.5f, model_msg->pose[agent_index].position.y - 0.5f));
                vertices.push_back(RVO::Vector2(model_msg->pose[agent_index].position.x + 0.5f, model_msg->pose[agent_index].position.y - 0.5f));
                vertices.push_back(RVO::Vector2(model_msg->pose[agent_index].position.x + 0.5f, model_msg->pose[agent_index].position.y + 0.5f));
                vertices.push_back(RVO::Vector2(model_msg->pose[agent_index].position.x - 0.5f, model_msg->pose[agent_index].position.y + 0.5f));
                sim->addObstacle(vertices);
                agent_name= "unit_box_" + std::to_string(i);
                
            }
        }

        // Process obstacles so that they are accounted for in the simulation.
        sim->processObstacles();
    }
    else
        std::cout << "error: please check the simulator" << std::endl;
}
void RVOPlanner::setPreferredVelocities()
{
    for (size_t i = 0; i < sim->getNumAgents(); ++i)
    {
        if (absSq(goals[i] - sim->getAgentPosition(i)) < goal_threshold)
        {
            // Agent is within one radius of its goal, set preferred velocity to zero
            sim->setAgentPrefVelocity(i, RVO::Vector2(0.0f, 0.0f));
        }
        else
        {
            sim->setAgentPrefVelocity(i, normalize(goals[i] - sim->getAgentPosition(i)));
        }
    }
}

void RVOPlanner::updateState_gazebo(gazebo_msgs::ModelStates::ConstPtr model_msg)
{
    if (simulator == "gazebo")
    {
        auto models_name = model_msg->name;
        int num = models_name.size();
        int count = 0;

        // sim->agents_.clear();

        for (int i = 0; i < num; i++)
        {
            std::string agent_name = "agent" + std::to_string(i + 1);

            auto iter_agent = std::find(models_name.begin(), models_name.end(), agent_name);
            int agent_index = iter_agent - models_name.begin();

            if (iter_agent != models_name.end())
            {
                float obs_x = model_msg->pose[agent_index].position.x;
                float obs_y = model_msg->pose[agent_index].position.y;
                float vel_x = model_msg->twist[agent_index].linear.x;
                float vel_y = model_msg->twist[agent_index].linear.y;

               if(IfInitial)
               {
                   sim->agents_[count]->position_ = RVO::Vector2(obs_x, obs_y);
                   sim->agents_[count]->velocity_ = RVO::Vector2(vel_x, vel_y);
               }                   
               else
                   sim->addAgent(RVO::Vector2(obs_x, obs_y));

                count++;
                // sim->agents_[i]->quater = model_msg->pose[agent_index].orientation;
            }
        }
    }
    else
        std::cout << "error: please check the simulator" << std::endl;
}

void RVOPlanner::setObstacles(const obstacle_detector::Obstacles new_obstacles)
{
    num_obstacle = 0;;
    sim->obstacles_.clear();

    for (int i = 0; i < new_obstacles.circles.size(); i++)
    {

        float obs_x = new_obstacles.circles[i].center.x;
        float obs_y = new_obstacles.circles[i].center.y;
        float vel_x = new_obstacles.circles[i].velocity.x;
        float vel_y = new_obstacles.circles[i].velocity.y;

        // Consider only static obstacles
        if(sqrt(pow(vel_x, 2) + pow(vel_y, 2)) >= 0.1)
            continue; 

        // Add (polygonal) obstacle(s), specifying vertices in counterclockwise order.
        std::vector<RVO::Vector2> vertices;
        vertices.push_back(RVO::Vector2(obs_x - new_obstacles.circles[i].radius, obs_y - new_obstacles.circles[i].radius));
        vertices.push_back(RVO::Vector2(obs_x + new_obstacles.circles[i].radius, obs_y - new_obstacles.circles[i].radius));
        vertices.push_back(RVO::Vector2(obs_x + new_obstacles.circles[i].radius, obs_y + new_obstacles.circles[i].radius));
        vertices.push_back(RVO::Vector2(obs_x - new_obstacles.circles[i].radius, obs_y + new_obstacles.circles[i].radius));
        sim->addObstacle(vertices);    
        num_obstacle++;      
    }

    // Process obstacles so that they are accounted for in the simulation.
    sim->processObstacles();    
}

void RVOPlanner::updateAgentStates(const obstacle_detector::Obstacles new_obstacles)
{ 
    num_agent = 0;
    sim->agents_.clear();
    goals.clear();

    for (int i = 0; i < new_obstacles.circles.size(); i++)
    {
        float obs_x = new_obstacles.circles[i].center.x;
        float obs_y = new_obstacles.circles[i].center.y;
        float vel_x = new_obstacles.circles[i].velocity.x;
        float vel_y = new_obstacles.circles[i].velocity.y;

        // Consider only dynamic obstacles
        if(sqrt(pow(vel_x, 2) + pow(vel_y, 2)) < 0.1)
            continue; 

        sim->addAgent(RVO::Vector2(obs_x, obs_y));

        sim->agents_[num_agent]->position_ = RVO::Vector2(obs_x, obs_y);
        sim->agents_[num_agent]->velocity_ = RVO::Vector2(vel_x, vel_y);
                           
        
        goals.emplace_back(Vector2(obs_x + 5*vel_x, obs_y + 5*vel_y));

        num_agent++;
    }
}


void RVOPlanner::updateRobotState(geometry_msgs::PoseWithCovarianceStamped amcl_pose, nav_msgs::Odometry odom)
{
    float obs_x = amcl_pose.pose.pose.position.x;
    float obs_y = amcl_pose.pose.pose.position.y;
    float obs_theta = tf::getYaw(amcl_pose.pose.pose.orientation);

    geometry_msgs::Vector3Stamped odom_vel, map_vel;
    odom_vel.header.stamp = ros::Time(0);
    odom_vel.header.frame_id = "odom";
    odom_vel.vector.x = odom.twist.twist.linear.x;
    odom_vel.vector.y = odom.twist.twist.linear.y;
    odom_vel.vector.z =0;
    
    tf::TransformListener transform_listen_;

    nav_msgs::Odometry odom_pose;
    odom_pose.pose.pose.position.x = odom.twist.twist.linear.x;
    odom_pose.pose.pose.position.y = odom.twist.twist.linear.y;    

    try{
        tf::StampedTransform transform;
        transform_listen_.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(10.0) );
        transform_listen_.lookupTransform("map", "base_footprint", ros::Time(0), transform);
        // transform_listen_.transformVector("map", odom_vel, map_vel);
        // transform_listen_.lookupTwist("base_link", "map", ros::Time::now(), ros::Duration(0.1), odom_twist_);
        poseTFToMsg(transform, odom_pose.pose.pose);
    }
    catch (tf::TransformException &ex) {
         ROS_ERROR("%s",ex.what());
    }

    size_t robot_id = sim->getNumAgents();
    sim->addAgent(RVO::Vector2(obs_x, obs_y));
    sim->agents_[robot_id]->position_ = RVO::Vector2(obs_x, obs_y);
    std::cout<<"### After conversion vel: " <<odom.twist.twist.linear.x*cos(tf::getYaw(odom_pose.pose.pose.orientation))<< ", "<<odom.twist.twist.linear.x*sin(tf::getYaw(odom_pose.pose.pose.orientation)) <<std::endl;
    //std::cout<<"### LookupTwist: " << odom_twist_.linear.x << ", "<< odom_twist_.linear.y <<std::endl;
    sim->agents_[robot_id]->velocity_ = RVO::Vector2(odom.twist.twist.linear.x*cos(tf::getYaw(odom_pose.pose.pose.orientation)), odom.twist.twist.linear.x*sin(tf::getYaw(odom_pose.pose.pose.orientation)) );

}

std::vector<RVO::Vector2 *> RVOPlanner::step()
{
    sim->kdTree_->buildAgentTree();
    newVelocities.clear();

    for (auto & agent : sim->agents_)
    {
        agent->computeNeighbors();
        agent->computeNewVelocity();
        auto *new_vel = new Vector2(agent->newVelocity_.x(), agent->newVelocity_.y());
        newVelocities.push_back(new_vel);
    }

    return newVelocities;
}

RVO::Vector2 RVOPlanner::getRobotCommand()
{
    std::cout<<"num_obstacle: " << num_obstacle << ", num_agent: " << num_agent <<std::endl;
    size_t robot_id = sim->getNumAgents();

    sim->kdTree_->buildAgentTree();

    if(robot_id)
    {
        sim->agents_[robot_id-1]->computeNeighbors(); 
        sim->agents_[robot_id-1]->computeNewVelocity();    
    }

    ROS_WARN("Calculated velocity: %f, %f", sim->agents_[robot_id-1]->newVelocity_.x(), sim->agents_[robot_id-1]->newVelocity_.y());
    return RVO::Vector2(sim->agents_[robot_id-1]->newVelocity_.x(), sim->agents_[robot_id-1]->newVelocity_.y());
}

}; // namespace RVO