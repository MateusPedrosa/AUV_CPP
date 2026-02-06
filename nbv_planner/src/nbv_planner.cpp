#include "nbv_planner/nbv_planner.hpp"
#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace nbv_planner {

NBVPlanner::NBVPlanner(rclcpp::Logger logger)
    : logger_(logger),
      sensor_range_(5.0),
      sensor_hfov_(M_PI / 2),  // 90 degrees
      sensor_vfov_(M_PI / 3),  // 60 degrees
      num_yaw_samples_(8),
      num_pitch_samples_(3),
      sampling_radius_(1.0)
{
}

void NBVPlanner::setParameters(
    double sensor_range,
    double sensor_hfov,
    double sensor_vfov,
    int num_yaw_samples,
    int num_pitch_samples)
{
    sensor_range_ = sensor_range;
    sensor_hfov_ = sensor_hfov;
    sensor_vfov_ = sensor_vfov;
    num_yaw_samples_ = num_yaw_samples;
    num_pitch_samples_ = num_pitch_samples;
}

geometry_msgs::msg::PoseStamped NBVPlanner::planNextBestView(
    const octomap::OcTree& octree,
    const geometry_msgs::msg::Pose& current_pose)
{
    RCLCPP_INFO(logger_, "Planning next best view...");
    
    candidates_.clear();
    
    // Generate candidate viewpoints
    auto candidate_poses = generateCandidates(current_pose);
    
    RCLCPP_INFO(logger_, "Evaluating %zu candidates", candidate_poses.size());
    
    // Evaluate each candidate
    double best_utility = -1.0;
    geometry_msgs::msg::Pose best_pose = current_pose;
    
    for (const auto& candidate : candidate_poses) {
        // Check validity
        if (!isViewpointValid(octree, candidate)) {
            continue;
        }
        
        // Calculate information gain
        double gain = calculateInformationGain(octree, candidate);
        
        // Calculate cost
        double cost = calculateCost(current_pose, candidate);
        
        // Calculate utility (simple gain/cost ratio)
        double utility = (cost > 0.01) ? (gain / cost) : 0.0;
        
        // Store candidate
        CandidateView view;
        view.pose = candidate;
        view.information_gain = gain;
        view.cost = cost;
        view.utility = utility;
        candidates_.push_back(view);
        
        // Update best
        if (utility > best_utility) {
            best_utility = utility;
            best_pose = candidate;
        }
    }
    
    RCLCPP_INFO(logger_, "Best utility: %.2f from %zu valid candidates", 
                best_utility, candidates_.size());
    
    geometry_msgs::msg::PoseStamped result;
    result.header.stamp = rclcpp::Clock().now();
    result.header.frame_id = "map";
    result.pose = best_pose;
    
    return result;
}

std::vector<geometry_msgs::msg::Pose> NBVPlanner::generateCandidates(
    const geometry_msgs::msg::Pose& current_pose)
{
    std::vector<geometry_msgs::msg::Pose> candidates;
    
    // Sample viewpoints in a sphere around current position
    for (int yaw_idx = 0; yaw_idx < num_yaw_samples_; ++yaw_idx) {
        double yaw = (2.0 * M_PI * yaw_idx) / num_yaw_samples_;
        
        for (int pitch_idx = 0; pitch_idx < num_pitch_samples_; ++pitch_idx) {
            // Sample pitch from -30 to +30 degrees
            double pitch = -M_PI/6 + (M_PI/3 * pitch_idx) / (num_pitch_samples_ - 1);
            
            geometry_msgs::msg::Pose candidate;
            
            // Position: offset from current position
            candidate.position.x = current_pose.position.x + 
                sampling_radius_ * std::cos(yaw) * std::cos(pitch);
            candidate.position.y = current_pose.position.y + 
                sampling_radius_ * std::sin(yaw) * std::cos(pitch);
            candidate.position.z = current_pose.position.z + 
                sampling_radius_ * std::sin(pitch);
            
            // Orientation: look at center
            tf2::Quaternion q;
            q.setRPY(0, pitch, yaw);
            candidate.orientation = tf2::toMsg(q);
            
            candidates.push_back(candidate);
        }
    }
    
    return candidates;
}

double NBVPlanner::calculateInformationGain(
    const octomap::OcTree& octree,
    const geometry_msgs::msg::Pose& viewpoint)
{
    octomap::point3d origin(
        viewpoint.position.x,
        viewpoint.position.y,
        viewpoint.position.z);
    
    // Extract yaw from quaternion
    tf2::Quaternion q(
        viewpoint.orientation.x,
        viewpoint.orientation.y,
        viewpoint.orientation.z,
        viewpoint.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    int total_unknown = 0;
    int num_rays = 20;  // Cast 20 rays per viewpoint
    
    // Cast rays in sensor's field of view
    for (int i = 0; i < num_rays; ++i) {
        double horizontal_angle = yaw - sensor_hfov_/2 + 
            (sensor_hfov_ * i) / (num_rays - 1);
        double vertical_angle = pitch;  // Simplified: only horizontal sweep
        
        octomap::point3d direction(
            sensor_range_ * std::cos(horizontal_angle) * std::cos(vertical_angle),
            sensor_range_ * std::sin(horizontal_angle) * std::cos(vertical_angle),
            sensor_range_ * std::sin(vertical_angle));
        
        total_unknown += countVisibleUnknownVoxels(octree, origin, direction);
    }
    
    return static_cast<double>(total_unknown);
}

int NBVPlanner::countVisibleUnknownVoxels(
    const octomap::OcTree& octree,
    const octomap::point3d& origin,
    const octomap::point3d& direction)
{
    octomap::point3d end = origin + direction;
    octomap::point3d ray_end;
    
    // Cast ray
    bool hit = octree.castRay(origin, direction, ray_end, true, sensor_range_);
    
    // Count unknown voxels along ray
    int unknown_count = 0;
    
    octomap::KeyRay key_ray;
    if (octree.computeRayKeys(origin, ray_end, key_ray)) {
        for (const auto& key : key_ray) {
            octomap::OcTreeNode* node = octree.search(key);
            if (node == nullptr) {
                // Unknown voxel
                unknown_count++;
            } else if (octree.isNodeOccupied(node)) {
                // Hit occupied voxel, stop counting
                break;
            }
        }
    }
    
    return unknown_count;
}

double NBVPlanner::calculateCost(
    const geometry_msgs::msg::Pose& from,
    const geometry_msgs::msg::Pose& to)
{
    // Simple Euclidean distance cost
    double dx = to.position.x - from.position.x;
    double dy = to.position.y - from.position.y;
    double dz = to.position.z - from.position.z;
    
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

bool NBVPlanner::isViewpointValid(
    const octomap::OcTree& octree,
    const geometry_msgs::msg::Pose& viewpoint)
{
    octomap::point3d point(
        viewpoint.position.x,
        viewpoint.position.y,
        viewpoint.position.z);
    
    // Check if viewpoint is in occupied space
    octomap::OcTreeNode* node = octree.search(point);
    if (node != nullptr && octree.isNodeOccupied(node)) {
        return false;
    }
    
    // Could add more checks: distance from obstacles, reachability, etc.
    
    return true;
}

} // namespace nbv_planner