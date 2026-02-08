#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/config.h>
#include <iostream>

#include "nbv_planner/simple_planner.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace nbv_planner {

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

void SimplePlanner::findPath(
    const OctomapManager& octomap_manager,
    const octomap::OcTree& octree,
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& goal)
{
    // std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    // Construct the state space we are planning in
    auto space(std::make_shared<ob::SE3StateSpace>());

    // Set the bounds for the R3 component of this state space
    ob::RealVectorBounds bounds(3);

    auto [mapMin, mapMax] = octomap_manager.getMapBounds();

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
    ss.setStateValidityChecker([this, &octree](const ob::State *state) { return isStateValid(state, octree); });

    auto planner = std::make_shared<og::RRTConnect>(ss.getSpaceInformation());
    
    // (Optional) tweak the planner
    planner->setRange(0.5);

    // Tell SimpleSetup to use this planner
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

    // Attempt to solve the problem within 1.0 second
    std::cout << "Starting planning..." << std::endl;
    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        
        // Simplify the solution (optimize the path)
        ss.simplifySolution();
        
        // Print the path to stdout (stream to console)
        ss.getSolutionPath().print(std::cout);
    }
    else
    {
        std::cout << "No solution found." << std::endl;
    }
}

} // namespace nbv_planner