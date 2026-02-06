#ifndef NBV_PLANNER_NBV_PLANNER_HPP
#define NBV_PLANNER_NBV_PLANNER_HPP

#include <vector>
#include <octomap/octomap.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace nbv_planner {

struct CandidateView {
    geometry_msgs::msg::Pose pose;
    double information_gain;
    double cost;
    double utility; // Combined metric (gain / cost)
};

class NBVPlanner {
public:
    explicit NBVPlanner(rclcpp::Logger logger);
    
    ~NBVPlanner() = default;

    /**
     * @brief Plan next best view
     * @param octree Current map
     * @param current_pose Robot's current pose
     * @return Best viewpoint to explore next
     */
    geometry_msgs::msg::PoseStamped planNextBestView(
        const octomap::OcTree& octree,
        const geometry_msgs::msg::Pose& current_pose);

    /**
     * @brief Set planning parameters
     */
    void setParameters(
        double sensor_range,
        double sensor_hfov,
        double sensor_vfov,
        int num_yaw_samples,
        int num_pitch_samples);

    /**
     * @brief Get all evaluated candidates (for visualization)
     */
    const std::vector<CandidateView>& getCandidates() const { 
        return candidates_; 
    }

private:
    /**
     * @brief Generate candidate viewpoints around current position
     */
    std::vector<geometry_msgs::msg::Pose> generateCandidates(
        const geometry_msgs::msg::Pose& current_pose);

    /**
     * @brief Calculate information gain for a viewpoint
     */
    double calculateInformationGain(
        const octomap::OcTree& octree,
        const geometry_msgs::msg::Pose& viewpoint);

    /**
     * @brief Calculate cost to reach viewpoint
     */
    double calculateCost(
        const geometry_msgs::msg::Pose& from,
        const geometry_msgs::msg::Pose& to);

    /**
     * @brief Check if viewpoint is collision-free
     */
    bool isViewpointValid(
        const octomap::OcTree& octree,
        const geometry_msgs::msg::Pose& viewpoint);

    /**
     * @brief Cast rays to compute visible unknown voxels
     */
    int countVisibleUnknownVoxels(
        const octomap::OcTree& octree,
        const octomap::point3d& origin,
        const octomap::point3d& direction);

    rclcpp::Logger logger_;
    
    // Planning parameters
    double sensor_range_;
    double sensor_hfov_;  // Horizontal field of view (radians)
    double sensor_vfov_;  // Vertical field of view (radians)
    int num_yaw_samples_;
    int num_pitch_samples_;
    double sampling_radius_;  // How far to sample candidates
    
    // Store candidates for visualization
    std::vector<CandidateView> candidates_;
};

} // namespace nbv_planner

#endif