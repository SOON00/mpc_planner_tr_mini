#include <pluginlib/class_list_macros.h>
#include <mpc_planner_rosnavigation/custom_global_planner.h>
#include <cmath>
#include <limits>

using namespace global_planner;

CustomGlobalPlanner::CustomGlobalPlanner()
    : initialized_(false), costmap_ros_(nullptr), current_waypoint_idx_(0), waypoint_interval_(1.5), pause_sent_(false) {}

CustomGlobalPlanner::CustomGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : initialized_(false), costmap_ros_(nullptr), current_waypoint_idx_(0), waypoint_interval_(1.5), pause_sent_(false) {
    initialize(name, costmap_ros);
}

void CustomGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (initialized_) return;

    costmap_ros_ = costmap_ros;
    ros::NodeHandle nh("~/" + name);

    path_sub_ = nh.subscribe("/topology_path", 1, &CustomGlobalPlanner::pathCallback, this);

    pause_pub_ = nh.advertise<std_msgs::Bool>("/pause_motion", 1);

    navfn_planner_.reset(new navfn::NavfnROS());
    navfn_planner_->initialize(name + "_navfn", costmap_ros_);

    initialized_ = true;
}

void CustomGlobalPlanner::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    external_plan_ = msg->poses;
    waypoints_.clear();
    current_waypoint_idx_ = 0;

    if (external_plan_.empty()) return;

    waypoints_.push_back(external_plan_.front());
    geometry_msgs::Pose last_wp = external_plan_.front().pose;

    for (const auto& pose : external_plan_) {
        double dx = pose.pose.position.x - last_wp.position.x;
        double dy = pose.pose.position.y - last_wp.position.y;
        if (std::hypot(dx, dy) >= waypoint_interval_) {
            waypoints_.push_back(pose);
            last_wp = pose.pose;
        }
    }

    if (waypoints_.back().pose.position != external_plan_.back().pose.position)
        waypoints_.push_back(external_plan_.back());
}

bool CustomGlobalPlanner::navfnFallback(const geometry_msgs::PoseStamped& start,
                                        const geometry_msgs::PoseStamped& goal,
                                        std::vector<geometry_msgs::PoseStamped>& plan) {
    std::vector<geometry_msgs::PoseStamped> navfn_plan;
    bool success = navfn_planner_->makePlan(start, goal, navfn_plan);
    if (success && !navfn_plan.empty()) {
        plan = navfn_plan;
        return true;
    }
    return false;
}

geometry_msgs::PoseStamped CustomGlobalPlanner::shiftGoalIfNecessary(
    const geometry_msgs::PoseStamped& goal) {

    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    unsigned int mx, my;

    if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my)) {
        ROS_WARN("Goal is out of costmap bounds.");
        return goal;
    }

    unsigned char cost = costmap->getCost(mx, my);
    if (cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return goal;  // Safe
    }

    // Try previous waypoints for safe shift
    for (int i = static_cast<int>(waypoints_.size()) - 2; i >= 0; --i) {
        const auto& wp = waypoints_[i].pose.position;
        if (costmap->worldToMap(wp.x, wp.y, mx, my) &&
            costmap->getCost(mx, my) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            geometry_msgs::PoseStamped shifted = goal;
            shifted.pose.position = wp;
            return shifted;
        }
    }

    ROS_WARN("No safe shift target found. Keeping original goal.");
    return goal;
}

geometry_msgs::PoseStamped CustomGlobalPlanner::shiftTargetWaypointIfNecessary(
    const geometry_msgs::PoseStamped& target_wp) {

    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    unsigned int mx, my;

    geometry_msgs::PoseStamped shifted_wp = target_wp;

    if (!costmap->worldToMap(target_wp.pose.position.x, target_wp.pose.position.y, mx, my)) {
        ROS_WARN("Target waypoint is out of costmap bounds.");
        return target_wp;
    }

    unsigned char cost = costmap->getCost(mx, my);
    if (cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        // Safe
        return target_wp;
    }

    // 위험하면 다음 waypoints 중 안전한 곳으로 shift 시도
    for (size_t i = current_waypoint_idx_ + 1; i < waypoints_.size(); ++i) {
        const auto& wp = waypoints_[i];
        if (!costmap->worldToMap(wp.pose.position.x, wp.pose.position.y, mx, my)) continue;

        if (costmap->getCost(mx, my) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            ROS_WARN("Shifted target waypoint from idx %lu to safe idx %lu", current_waypoint_idx_, i);
            shifted_wp = wp;
            current_waypoint_idx_ = i;  // 인덱스도 이동
            break;
        }
    }

    return shifted_wp;
}

bool CustomGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                   const geometry_msgs::PoseStamped& goal,
                                   std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_ || waypoints_.empty()) {
        ROS_WARN("Planner not initialized or no waypoints.");
        return false;
    }

    costmap_ros_->updateMap();
    ros::spinOnce();

    geometry_msgs::PoseStamped target_wp = waypoints_[current_waypoint_idx_];

    // target_wp 위험 여부 확인 후 필요시 shift
    target_wp = shiftTargetWaypointIfNecessary(target_wp);

    double dx = target_wp.pose.position.x - start.pose.position.x;
    double dy = target_wp.pose.position.y - start.pose.position.y;
    double dist_to_waypoint = std::hypot(dx, dy);

    if (dist_to_waypoint < 1.0 && current_waypoint_idx_ < waypoints_.size() - 1) {
        current_waypoint_idx_++;
        target_wp = waypoints_[current_waypoint_idx_];
        ROS_INFO("Switching to next waypoint: %lu", current_waypoint_idx_);
    }

    geometry_msgs::PoseStamped safe_goal = shiftGoalIfNecessary(goal);

    std::vector<geometry_msgs::PoseStamped> partial_plan;
    if (!navfnFallback(start, target_wp, partial_plan)) {
        ROS_WARN("Navfn fallback failed.");
        return false;
    }

    double total_length = 0.0;
    for (size_t i = 1; i < partial_plan.size(); ++i) {
        const auto& p1 = partial_plan[i - 1].pose.position;
        const auto& p2 = partial_plan[i].pose.position;
        total_length += std::hypot(p2.x - p1.x, p2.y - p1.y);
    }

    /*--------------- STOCKING ALGORITHM ---------------*/
    // std_msgs::Bool pause_msg;

    // if (total_length >= 10.0) {
    //     if (!pause_sent_) {
    //         // 1회성으로 true 발행
    //         pause_msg.data = true;
    //         pause_pub_.publish(pause_msg);
    //         pause_sent_ = true;
    //         ROS_INFO("Pause ON sent.");
    //     } else {
    //         // 이미 true를 보냈으면 false를 보내서 해제 신호 주기
    //         pause_msg.data = false;
    //         pause_pub_.publish(pause_msg);
    //     }
    // } else {
    //     // 길이가 짧으면 항상 false
    //     pause_msg.data = false;
    //     pause_pub_.publish(pause_msg);
    //     pause_sent_ = false;  // 상태 초기화
    // }
    /*---------------------------------------------------*/

    plan.clear();
    plan.insert(plan.end(), partial_plan.begin(), partial_plan.end());

    geometry_msgs::PoseStamped end_pose;
    if (current_waypoint_idx_ < waypoints_.size() - 1) {
        end_pose = waypoints_[current_waypoint_idx_ + 1];
    } else {
        end_pose = safe_goal;
    }

    if (!plan.empty()) {
        plan.back() = end_pose;
    } else {
        plan.push_back(end_pose);
    }

    return true;
}

PLUGINLIB_EXPORT_CLASS(global_planner::CustomGlobalPlanner, nav_core::BaseGlobalPlanner)
