#include "waypoint_global_planner/interpolate_global_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

PLUGINLIB_EXPORT_CLASS(interpolate_global_planner::InterPolateGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace interpolate_global_planner
{
  InterPolateGlobalPlanner::InterPolateGlobalPlanner() : costmap_ros_(NULL), initialized_(false) {}

  InterPolateGlobalPlanner::InterPolateGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros);
  }

  InterPolateGlobalPlanner::~InterPolateGlobalPlanner() {}

  void InterPolateGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    if(!initialized_)
    {
      // get the costmap
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      world_model_ = new base_local_planner::CostmapModel(*costmap_);
      global_frame_ = costmap_ros_->getGlobalFrameID();

      ros::NodeHandle nh;
      ros::NodeHandle pnh("~" + name);

      // initialize publishers and subscribers
      resolution_ = 0.05;
      plan_pub_ = pnh.advertise<nav_msgs::Path>("global_plan", 1);
      // pnh.param("resolution",resolution_);

      initialized_ = true;
      ROS_INFO("Planner has been initialized");
    }
    else
    {
      ROS_WARN("This planner has already been initialized");
    }
  }


  bool InterPolateGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start_pose,
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
  {
    nav_msgs::Path interpolated_plan;
    interpolated_plan.header.frame_id = global_frame_;
    interpolated_plan.header.stamp = ros::Time::now();
    interpolatePath(interpolated_plan, plan, start_pose, goal);
    plan_pub_.publish(interpolated_plan);
    return true;
  }

  void InterPolateGlobalPlanner::interpolatePath(nav_msgs::Path& path,
      std::vector<geometry_msgs::PoseStamped>& plan,
      const geometry_msgs::PoseStamped& start_pose,
      const geometry_msgs::PoseStamped& goal_pose)
  {
    double dx = goal_pose.pose.position.x - start_pose.pose.position.x;
    double dy = goal_pose.pose.position.y - start_pose.pose.position.y;
    double yaw = atan2(dy, dx);
    double dist = hypot(dx, dy);
    int waypoints = static_cast<int>(dist/resolution_);
    double increase_x = dx/waypoints;
    double increase_y = dy/waypoints;
    tf2::Quaternion quat_temp;
    quat_temp.setRPY(0,0,yaw);
    

    geometry_msgs::PoseStamped temp_pose;
    temp_pose.header.frame_id = path.header.frame_id;
    temp_pose.header.stamp = path.header.stamp;
    for (int i=0; i < waypoints; i++){
      // calculate distance between two consecutive waypoints
      temp_pose.header.seq = i;
      temp_pose.pose.position.x = start_pose.pose.position.x + i*increase_x;
      temp_pose.pose.position.y = start_pose.pose.position.y + i*increase_y;
      temp_pose.pose.orientation.x = quat_temp[0];
      temp_pose.pose.orientation.y = quat_temp[1];
      temp_pose.pose.orientation.z = quat_temp[2];
      temp_pose.pose.orientation.w = quat_temp[3];
      plan.push_back(temp_pose);
      path.poses.push_back(temp_pose);
    }
    plan.push_back(goal_pose);
    plan.back().header.seq = waypoints;
    path.poses.push_back(goal_pose);
    path.poses.back().header.seq = waypoints;
  }

}  // namespace waypoint_global_planner
