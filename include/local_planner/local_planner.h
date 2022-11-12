#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

 using std::string;


 namespace local_planner {

 class LocalPlanner : public nav_core::BaseLocalPlanner {
 public:

  LocalPlanner();
    LocalPlanner(std::string name, tf2_ros::Buffer* tf,costmap_2d::Costmap2DROS* costmap_ros);
    ~LocalPlanner();

    void initialize(std::string name, tf2_ros::Buffer* tf,costmap_2d::Costmap2DROS* costmap_ros);
  
   bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();
private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    tf2_ros::Buffer* tf_;
    bool initialized_;
  };
 };
 #endif

