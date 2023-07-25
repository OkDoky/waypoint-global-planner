#ifndef INTERPOLATE_GLOBAL_PLANNER_H
#define INTERPOLATE_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>

namespace interpolate_global_planner
{

/**
 * @class CarrotPlanner
 * @brief Provides a simple global planner for producing boustrophedon paths without taking into account obstacles
 */
class InterPolateGlobalPlanner : public nav_core::BaseGlobalPlanner
{
  public:
    /**
     * @brief Default Constructor
     */
    InterPolateGlobalPlanner();

    /**
     * @brief Constructor for the planner
     * @param name The name of this planner
     * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
    InterPolateGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief Default destructor
     */
    ~InterPolateGlobalPlanner();

    /**
     * @brief Initialization function for the planner
     * @param name The name of this planner
     * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief Given a goal pose in the world, compute a plan
     * @param start_pose The starting pose of the robot
     * @param goal The goal pose
     * @param plan The plan filled by the planner
     * @return True if a valid plan was found, false otherwise
     */
    bool makePlan(const geometry_msgs::PoseStamped& start_pose,
      const geometry_msgs::PoseStamped& goal,
      std::vector<geometry_msgs::PoseStamped>& plan);

    /**
     * @brief Interpolates a path (position and orientation) using a fixed number of points per meter
     * @param path The input path to be interpolated
     */
    void interpolatePath(nav_msgs::Path& path,
      std::vector<geometry_msgs::PoseStamped>& plan,
      const geometry_msgs::PoseStamped& start_pose,
      const geometry_msgs::PoseStamped& goal_pose);

  private:
    double resolution_;
    bool initialized_;  //!< flag indicating the planner has been initialized
    std::string global_frame_;
    costmap_2d::Costmap2DROS* costmap_ros_;  //!< costmap ros wrapper
    costmap_2d::Costmap2D* costmap_;  //!< costmap container
    base_local_planner::WorldModel* world_model_;  //!< world model

    // subscribers and publishers
    ros::Publisher plan_pub_;  //!< publisher of the global plan

    // containers
    nav_msgs::Path path_;  //!< container for the generated interpolated path
};

}  // namespace waypoint_global_planner


#endif /* INTERPOLATE_GLOBAL_PLANNER_H */
