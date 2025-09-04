#ifndef MPC_PLANNER_ROSNAVIGATION_CUSTOM_GLOBAL_PLANNER_H
#define MPC_PLANNER_ROSNAVIGATION_CUSTOM_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <navfn/navfn_ros.h>

#include <vector>
#include <string>

namespace global_planner {

class CustomGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
    CustomGlobalPlanner();
    CustomGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    bool navfnFallback(const geometry_msgs::PoseStamped& start,
                       const geometry_msgs::PoseStamped& goal,
                       std::vector<geometry_msgs::PoseStamped>& plan);

    geometry_msgs::PoseStamped shiftGoalIfNecessary(const geometry_msgs::PoseStamped& goal);
    geometry_msgs::PoseStamped shiftTargetWaypointIfNecessary(const geometry_msgs::PoseStamped& target_wp);

private:
    bool initialized_;
    costmap_2d::Costmap2DROS* costmap_ros_;

    ros::Subscriber path_sub_;
    ros::Publisher pause_pub_;

    std::unique_ptr<navfn::NavfnROS> navfn_planner_;

    std::vector<geometry_msgs::PoseStamped> external_plan_;
    std::vector<geometry_msgs::PoseStamped> waypoints_;

    size_t current_waypoint_idx_;
    double waypoint_interval_;

    bool pause_sent_;
};

}  // namespace global_planner

#endif  // MPC_PLANNER_ROSNAVIGATION_CUSTOM_GLOBAL_PLANNER_H
