#ifndef NAV_ROV_H_
#define NAV_ROV_H_

#include "Agent.h"
#include "KdTree.h"
#include "Definitions.h"
#include "Obstacle.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Point.h"
#include "RVOSimulator.h"
#include <string>
#include <random>

#include <obstacle_detector/Obstacles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>


namespace RVO {

    class Agent;
    class Obstacle;
    class KdTree;

    class RVOPlanner{
    public:
        RVOPlanner(std::string simulation);

        void setupScenario(float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed);

        void updateState_gazebo(gazebo_msgs::ModelStates::ConstPtr model_msg);
        

        void setGoal();
        void randGoal(const float limit_goal[4], const std::string &model="default");
        void randomOnceGoal(const float limit_goal[4]);
        bool arrived();
        void setGoal(std::vector<geometry_msgs::Point> set_goals);
        void setInitial();
        void setPreferredVelocities();
        void setObstacles(gazebo_msgs::ModelStates::ConstPtr model_msg);

        std::vector<RVO::Vector2*>  step();
        float goal_threshold = 0.03;
        
        // Move base, obstacle detector
        unsigned int num_obstacle, num_agent;
        obstacle_detector::Obstacles raw_obstacles_;
        void updateAgentStates(const obstacle_detector::Obstacles::ConstPtr);
        void updateRobotState(geometry_msgs::PoseWithCovarianceStamped, nav_msgs::Odometry);
        void setObstacles(const obstacle_detector::Obstacles::ConstPtr);
        RVO::Vector2 getRobotCommand();  
        
        
    private:

        RVO::RVOSimulator* sim;
        std::string simulator;
        std::vector <RVO::Vector2> goals;
        bool IfInitial = false;
        std::vector<RVO::Vector2 *> newVelocities;
        

        friend class Agent;
        friend class KdTree;
        friend class Obstacle;
    };
}

#endif