#ifndef NBV_PLANNER_SIMPLE_PLANNER_HPP
#define NBV_PLANNER_SIMPLE_PLANNER_HPP

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/config.h>

#include <octomap/octomap.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/path.hpp>


#include "nbv_planner/action/plan_path.hpp"
#include "nbv_planner/octomap_manager.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace nbv_planner {

class SimplePlanner  : public rclcpp::Node
{
public:
    using PlanPath = nbv_planner::action::PlanPath;
    using GoalHandlePlanPath = rclcpp_action::ServerGoalHandle<PlanPath>;

    explicit SimplePlanner(
        rclcpp::Node* node,
        std::shared_ptr<OctomapManager> octomap_manager
    );
    
    ~SimplePlanner() = default;

private:
    // Reference to parent node for logging and time
    rclcpp::Node* node_;
    
    // Shared pointer to octomap manager (shared with NBVPlannerNode)
    std::shared_ptr<OctomapManager> octomap_manager_;

    // Action server
    rclcpp_action::Server<PlanPath>::SharedPtr action_server_;

    // Publisher for path visualization
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Action server callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const PlanPath::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePlanPath> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandlePlanPath> goal_handle);

    void execute(const std::shared_ptr<GoalHandlePlanPath> goal_handle);

    /**
     * @brief Check if state is collision-free
     */
    bool isStateValid(const ob::State *state, const octomap::OcTree& octree);

    /**
     * @brief Find a path from start to goal using the given octree
     */
    bool findPath(
        const octomap::OcTree& octree,
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::Pose& goal,
        nav_msgs::msg::Path& result_path,
        const std::shared_ptr<GoalHandlePlanPath>& goal_handle,
        double time_limit);

    /**
     * @brief Convert an OMPL path to a ROS path message
     */
    nav_msgs::msg::Path omplPathToRosPath(
        const og::PathGeometric& ompl_path,
        const std::string& frame_id = "map");
};

} // namespace nbv_planner

#endif