#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <local_planner/local_planner.h>

#include <tf/transform_broadcaster.h>

 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlanner, nav_core::BaseLocalPlanner)
 
 using namespace std;

 //Default Constructor
 namespace local_planner {

LocalPlanner::LocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {
cout<<"Hello";
}

LocalPlanner::LocalPlanner(std::string name, tf2_ros::Buffer* tf,costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false)
{
    initialize(name, tf, costmap_ros);
}

void LocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf,costmap_2d::Costmap2DROS* costmap_ros)
{
cout<<"Hello local";
    if(!initialized_)
    {
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        initialized_ = true;
    cout<<"Got local plan";
    }
}

bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    return true;
}

bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    else {
    cmd_vel.linear.x=1;
    cmd_vel.linear.y=0;
    cmd_vel.linear.z=0;
    cmd_vel.angular.x=0;
    cmd_vel.angular.y=0;
    cmd_vel.angular.z=0;
    cout<<"Local";
    }
    return true;
}

bool LocalPlanner::isGoalReached()
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    return false;
}
 };
