#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/config.h>
#include <iostream>

#include "nbv_planner/simple_planner.hpp"

using namespace std::placeholders;

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace nbv_planner {

SimplePlanner::SimplePlanner(
    rclcpp::Node* node,
    std::shared_ptr<OctomapManager> octomap_manager)
    : rclcpp::Node("simple_planner_node"),
      node_(node),
      octomap_manager_(octomap_manager)
{
    RCLCPP_INFO(this->get_logger(), "Initializing SimplePlanner action server...");

    // Create the action server
    this->action_server_ = rclcpp_action::create_server<PlanPath>(
        node_,
        "plan_path",
        std::bind(&SimplePlanner::handle_goal, this, _1, _2),
        std::bind(&SimplePlanner::handle_cancel, this, _1),
        std::bind(&SimplePlanner::handle_accepted, this, _1));

    // Create publisher for path visualization
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("planned_path", 10);

    RCLCPP_INFO(this->get_logger(), "SimplePlanner action server ready");
}

rclcpp_action::GoalResponse SimplePlanner::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PlanPath::Goal> goal)
{
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    
    // Add validation here
    // For example, check if start and goal poses are valid
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SimplePlanner::handle_cancel(
    const std::shared_ptr<GoalHandlePlanPath> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SimplePlanner::handle_accepted(const std::shared_ptr<GoalHandlePlanPath> goal_handle)
{
    // This needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&SimplePlanner::execute, this, _1), goal_handle}.detach();
}

void SimplePlanner::execute(const std::shared_ptr<GoalHandlePlanPath> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    
    const auto goal = goal_handle->get_goal();
    double time_limit = (goal->time_limit > 0.0) ? goal->time_limit : 1.0;
    auto feedback = std::make_shared<PlanPath::Feedback>();
    auto result = std::make_shared<PlanPath::Result>();

    // Send initial feedback
    feedback->status = "Initializing planning with time limit: " + std::to_string(time_limit) + " seconds";
    feedback->progress = 0.0;
    goal_handle->publish_feedback(feedback);

    if (!octomap_manager_) {
        RCLCPP_ERROR(this->get_logger(), "Octomap manager not initialized");
        result->success = false;
        result->message = "Octomap manager not initialized";
        goal_handle->abort(result);
        return;
    }

    const octomap::OcTree& octree = octomap_manager_->getOctree();

    // Update feedback
    feedback->status = "Planning path";
    feedback->progress = 0.3;
    goal_handle->publish_feedback(feedback);

    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Goal canceled";
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
    }

    // Plan the path
    nav_msgs::msg::Path ros_path;
    bool planning_success = findPath(
        octree,
        goal->start_pose,
        goal->goal_pose,
        ros_path,
        goal_handle,
        time_limit);

    // Update feedback
    feedback->status = "Planning complete";
    feedback->progress = 1.0;
    goal_handle->publish_feedback(feedback);

    // Set result
    result->success = planning_success;
    result->path = ros_path;
    
    if (planning_success) {
        result->message = "Path planning succeeded";
        path_pub_->publish(ros_path);
        RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints to /planned_path topic", ros_path.poses.size());
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    } else {
        result->message = "Path planning failed - no solution found";
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Goal aborted");
    }
}

bool SimplePlanner::isStateValid(const ob::State *state, const octomap::OcTree& octree)
{
    // Cast the abstract state type to the type we expect (SE3StateSpace)
    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

    // Extract the R3 (position) component
    const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    // Extract the SO3 (rotation) component
    // const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    // Check if the position is in occupied space
    octomap::point3d point(pos->values[0], pos->values[1], pos->values[2]);
    octomap::OcTreeNode* node = octree.search(point);
    if (node == nullptr || octree.isNodeOccupied(node)) {
        return false;
    }
    
    return true; 
}

bool SimplePlanner::findPath(
    const octomap::OcTree& octree,
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& goal,
    nav_msgs::msg::Path& result_path,
    const std::shared_ptr<GoalHandlePlanPath>& goal_handle,
    double time_limit)
{
    // Construct the state space we are planning in
    auto space(std::make_shared<ob::SE3StateSpace>());

    // Set the bounds for the R3 component of this state space
    ob::RealVectorBounds bounds(3);

    // Get map bounds from octomap manager
    auto [mapMin, mapMax] = octomap_manager_->getMapBounds();

    bounds.setLow(0, mapMin.x());
    bounds.setHigh(0, mapMax.x());
    bounds.setLow(1, mapMin.y());
    bounds.setHigh(1, mapMax.y());
    bounds.setLow(2, mapMin.z());
    bounds.setHigh(2, mapMax.z());

    space->setBounds(bounds);

    // Create an instance of SimpleSetup
    og::SimpleSetup ss(space);

    // Set the state validity checker
    // We use a lambda to map the function pointer to the std::function expected by OMPL
    ss.setStateValidityChecker([this, &octree](const ob::State *state) {
        return isStateValid(state, octree);
    });

    auto planner = std::make_shared<og::RRTConnect>(ss.getSpaceInformation());
    planner->setRange(0.5);
    ss.setPlanner(planner);

    ob::ScopedState<ob::SE3StateSpace> start_state(space);
    start_state->setXYZ(start.position.x, start.position.y, start.position.z);
    start_state->rotation().x = start.orientation.x;
    start_state->rotation().y = start.orientation.y;
    start_state->rotation().z = start.orientation.z;
    start_state->rotation().w = start.orientation.w;

    ob::ScopedState<ob::SE3StateSpace> goal_state(space);
    goal_state->setXYZ(goal.position.x, goal.position.y, goal.position.z);
    goal_state->rotation().x = goal.orientation.x;
    goal_state->rotation().y = goal.orientation.y;
    goal_state->rotation().z = goal.orientation.z;
    goal_state->rotation().w = goal.orientation.w;

    // Set these states in SimpleSetup
    ss.setStartAndGoalStates(start_state, goal_state);

    // Check for cancellation before planning
    if (goal_handle && goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "Planning canceled before starting");
        return false;
    }

    // Attempt to solve the problem within time limit
    RCLCPP_INFO(this->get_logger(), "Starting path planning...");
    RCLCPP_INFO(this->get_logger(), "Planning Bounds: X[%f, %f], Y[%f, %f], Z[%f, %f]", 
            bounds.low[0], bounds.high[0], bounds.low[1], bounds.high[1], bounds.low[2], bounds.high[2]);
    ob::PlannerStatus solved = ss.solve(time_limit);

    if (solved)
    {
        RCLCPP_INFO(this->get_logger(), "Found path solution:");
        
        // Simplify the solution (optimize the path)
        ss.simplifySolution();
        
        // Convert to ROS path
        result_path = omplPathToRosPath(ss.getSolutionPath(), "map");

        return true;
    }
    RCLCPP_WARN(this->get_logger(), "No path solution found.");

    return false;
}

nav_msgs::msg::Path SimplePlanner::omplPathToRosPath(
    const og::PathGeometric& ompl_path,
    const std::string& frame_id)
{
    nav_msgs::msg::Path ros_path;
    ros_path.header.stamp = this->now();
    ros_path.header.frame_id = frame_id;

    // Convert each state in the OMPL path to a PoseStamped
    for (size_t i = 0; i < ompl_path.getStateCount(); ++i)
    {
        const auto *se3state = ompl_path.getState(i)->as<ob::SE3StateSpace::StateType>();
        
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->now();
        pose_stamped.header.frame_id = frame_id;
        
        // Position
        const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
        pose_stamped.pose.position.x = pos->values[0];
        pose_stamped.pose.position.y = pos->values[1];
        pose_stamped.pose.position.z = pos->values[2];
        
        // Orientation
        const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
        pose_stamped.pose.orientation.x = rot->x;
        pose_stamped.pose.orientation.y = rot->y;
        pose_stamped.pose.orientation.z = rot->z;
        pose_stamped.pose.orientation.w = rot->w;
        
        ros_path.poses.push_back(pose_stamped);
    }

    return ros_path;
}

} // namespace nbv_planner