#ifndef NBV_PLANNER_SIMPLE_PLANNER_HPP
#define NBV_PLANNER_SIMPLE_PLANNER_HPP

#include <octomap/octomap.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include "nbv_planner/octomap_manager.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace nbv_planner {

class SimplePlanner {
public:
    explicit SimplePlanner(rclcpp::Logger logger);
    
    ~SimplePlanner() = default;

    void findPath(
        const OctomapManager& octomap_manager,
        const octomap::OcTree& octree,
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::Pose& goal
    );

private:
    /**
     * @brief Check if state is collision-free
     */
    bool isStateValid(const ob::State *state, const octomap::OcTree& octree);
};

} // namespace nbv_planner

#endif