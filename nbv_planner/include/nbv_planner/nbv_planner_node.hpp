#ifndef NBV_PLANNER_NBV_PLANNER_NODE_HPP
#define NBV_PLANNER_NBV_PLANNER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "nbv_planner/octomap_manager.hpp"
#include "nbv_planner/nbv_planner.hpp"

namespace nbv_planner {

class NBVPlannerNode : public rclcpp::Node {
public:
    NBVPlannerNode();
    
    ~NBVPlannerNode() = default;

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    void planningTimerCallback();
    
    void publishOctomapMarkers();
    
    void publishCandidateMarkers();

    void publishFrustumMarker();

    void publishSecondaryFrustumMarker();
    
    // Core components
    std::unique_ptr<OctomapManager> octomap_manager_;
    std::unique_ptr<NBVPlanner> nbv_planner_;
    
    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidates_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frustum_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr secondary_frustum_pub_;
    rclcpp::TimerBase::SharedPtr planning_timer_;
    
    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Parameters
    std::string map_frame_;
    std::string robot_frame_;
    std::string cloud_topic_;
    double planning_frequency_;
    std::string secondary_sensor_frame_;
    CameraIntrinsics camera_intrinsics_;
    
    // State
    geometry_msgs::msg::Pose current_pose_;
    bool received_first_cloud_;
};

} // namespace nbv_planner

#endif