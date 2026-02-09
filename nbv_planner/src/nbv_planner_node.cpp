#include "nbv_planner/nbv_planner_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "tf2_ros/create_timer_ros.h"

namespace nbv_planner {

NBVPlannerNode::NBVPlannerNode()
    : Node("nbv_planner_node"),
      received_first_cloud_(false)
{
    // Declare parameters
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("robot_frame", "base_link");
    this->declare_parameter("octree_resolution", 0.1);
    this->declare_parameter("planning_frequency", 0.5);
    this->declare_parameter("exploration_sensor_max_range", 5.0);
    this->declare_parameter("exploration_sensor_min_range", 0.1);
    this->declare_parameter("max_free_space", 0.0);
    this->declare_parameter("min_height_free_space", 0.0);
    this->declare_parameter("exploration_sensor_hfov", 130.0 * M_PI / 180.0);
    this->declare_parameter("exploration_sensor_vfov", 20.0 * M_PI / 180.0); // 60 degrees
    this->declare_parameter("inspection_sensor_frames", std::vector<std::string>{"camera_forward_link", "camera_bottom_link"});
    this->declare_parameter("exploration_sensor_frame", "sonar_link");
    this->declare_parameter("camera_fx", 525.0);
    this->declare_parameter("camera_fy", 525.0);
    this->declare_parameter("camera_cx", 319.5);
    this->declare_parameter("camera_cy", 239.5);
    this->declare_parameter("camera_width", 640);
    this->declare_parameter("camera_height", 480);
    this->declare_parameter("camera_max_range", 5.0);
    
    // Get parameters
    map_frame_ = this->get_parameter("map_frame").as_string();
    robot_frame_ = this->get_parameter("robot_frame").as_string();
    planning_frequency_ = this->get_parameter("planning_frequency").as_double();

    inspection_sensor_frames_ = this->get_parameter("inspection_sensor_frames").as_string_array();
    exploration_sensor_frame_ = this->get_parameter("exploration_sensor_frame").as_string();
    
    camera_intrinsics_ = {
        this->get_parameter("camera_fx").as_double(),
        this->get_parameter("camera_fy").as_double(),
        this->get_parameter("camera_cx").as_double(),
        this->get_parameter("camera_cy").as_double(),
        static_cast<int>(this->get_parameter("camera_width").as_int()),
        static_cast<int>(this->get_parameter("camera_height").as_int()),
        this->get_parameter("camera_max_range").as_double()
    };

    // Configure octomap parameters
    OctomapParameters octomap_params;
    octomap_params.resolution = this->get_parameter("octree_resolution").as_double();
    octomap_params.sensor_max_range = this->get_parameter("exploration_sensor_max_range").as_double();
    octomap_params.sensor_min_range = this->get_parameter("exploration_sensor_min_range").as_double();
    octomap_params.max_free_space = this->get_parameter("max_free_space").as_double();
    octomap_params.min_height_free_space = this->get_parameter("min_height_free_space").as_double();
    octomap_params.sensor_hfov = this->get_parameter("exploration_sensor_hfov").as_double();
    octomap_params.sensor_vfov = this->get_parameter("exploration_sensor_vfov").as_double();
    
    // Initialize components
    octomap_manager_ = std::make_shared<OctomapManager>(octomap_params);
    nbv_planner_ = std::make_unique<NBVPlanner>(this->get_logger());

    // Initialize path planner action server with shared octomap_manager
    path_planner_ = std::make_unique<SimplePlanner>(this, octomap_manager_);
    
    // Configure planner (placeholder)
    nbv_planner_->setParameters(octomap_params.sensor_max_range, M_PI/2, M_PI/3, 8, 3);
    
    // Initialize TF2
    std::chrono::milliseconds buffer_timeout(100); // 100 ms timeout for TF2 lookups
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    
    // Create subscribers
    point_cloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>();
    point_cloud_sub_->subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
    tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        *point_cloud_sub_, 
        *tf2_buffer_, 
        map_frame_, 
        10,
        this->get_node_logging_interface(),
        this->get_node_clock_interface(),
        buffer_timeout
    );
    tf2_filter_->registerCallback(&NBVPlannerNode::pointCloudCallback, this);
    
    // Create publishers
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "nbv_goal", 10);
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "octomap_markers", 10);
    candidates_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "candidate_markers", 10);
    frustum_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("exploration_sensor_frustum", 10);
    inspection_frustum_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("inspection_sensor_frustum", 10);
    
    // Create timer for planning
    auto period = std::chrono::duration<double>(1.0 / planning_frequency_);
    planning_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&NBVPlannerNode::planningTimerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "NBV Planner Node initialized");
    RCLCPP_INFO(this->get_logger(), "  Resolution: %.3f m", octomap_params.resolution);
    RCLCPP_INFO(this->get_logger(), "  Exploration sensor range: [%.2f, %.2f] m", 
                octomap_params.sensor_min_range, octomap_params.sensor_max_range);
}

void NBVPlannerNode::publishFrustumMarker() {
    const auto& params = octomap_manager_->getParams();
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = exploration_sensor_frame_; //TODO: get frame from params
    marker.header.stamp = rclcpp::Time(0);
    marker.frame_locked = true;
    marker.ns = "frustum";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02; // Line thickness

    // Calculate frustum corners based on your planner params
    float range = params.sensor_max_range;
    float h_half = range * tan(params.sensor_hfov / 2.0);
    float v_half = range * tan(params.sensor_vfov / 2.0);

    // Define the 5 points: Origin (0,0,0) and 4 corners of the far plane
    geometry_msgs::msg::Point p0, p1, p2, p3, p4;
    p0.x = 0; p0.y = 0; p0.z = 0;
    p1.x = range; p1.y = h_half; p1.z = v_half;
    p2.x = range; p2.y = -h_half; p2.z = v_half;
    p3.x = range; p3.y = -h_half; p3.z = -v_half;
    p4.x = range; p4.y = h_half; p4.z = -v_half;

    // Add lines from origin to corners and connecting corners...
    marker.points = {p0, p1, p0, p2, p0, p3, p0, p4, p1, p2, p2, p3, p3, p4, p4, p1};
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    frustum_pub_->publish(marker);
}

void NBVPlannerNode::publishInspectionFrustumMarker(const std::string& frame_id) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Time(0);
    marker.frame_locked = true;
    marker.ns = frame_id + "_frustum";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02; // Line thickness

    FrustumAngles frustum_angles = octomap_manager_->getCameraFOV(camera_intrinsics_);
    float range = camera_intrinsics_.max_range;
    float h_half = frustum_angles.horizontal / 2.0 * range;
    float v_half = frustum_angles.vertical / 2.0 * range;

    // Define the 5 points: Origin (0,0,0) and 4 corners of the far plane
    geometry_msgs::msg::Point p0, p1, p2, p3, p4;
    p0.x = 0; p0.y = 0; p0.z = 0;
    p1.x = range; p1.y = h_half; p1.z = v_half;
    p2.x = range; p2.y = -h_half; p2.z = v_half;
    p3.x = range; p3.y = -h_half; p3.z = -v_half;
    p4.x = range; p4.y = h_half; p4.z = -v_half;

    // Add lines from origin to corners and connecting corners...
    marker.points = {p0, p1, p0, p2, p0, p3, p0, p4, p1, p2, p2, p3, p3, p4, p4, p1};
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    inspection_frustum_pub_->publish(marker);
}

void NBVPlannerNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (msg->header.frame_id != exploration_sensor_frame_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,  // Log every 5 seconds max
            "Received point cloud in frame '%s', expected '%s'",
            msg->header.frame_id.c_str(), 
            exploration_sensor_frame_.c_str());
        return;
    }
    try {
        // Get transform from exploration sensor to map frame
        geometry_msgs::msg::TransformStamped t_exploration_sensor;
        t_exploration_sensor = tf2_buffer_->lookupTransform(
            map_frame_,
            msg->header.frame_id, // Point cloud should be published in the exploration sensor frame
            msg->header.stamp);
        // Convert to Eigen transform
        Eigen::Isometry3d T_G_exploration_sensor = tf2::transformToEigen(t_exploration_sensor);

        // Convert ROS PointCloud2 to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);
        
        // Update octree using batch ray casting for the exploration sensor
        octomap_manager_->insertPointCloudIntoMap(cloud, T_G_exploration_sensor);

        for (const auto& inspection_sensor_frame : inspection_sensor_frames_) {
            // Get transform from inspection sensor to map frame
            geometry_msgs::msg::TransformStamped t_inspection_sensor;
            t_inspection_sensor = tf2_buffer_->lookupTransform(
                map_frame_,
                inspection_sensor_frame,
                msg->header.stamp,
                tf2::durationFromSec(0.1)); // 100 ms timeout for this lookup
            // Convert to Eigen transform
            Eigen::Isometry3d T_G_inspection_sensor = tf2::transformToEigen(t_inspection_sensor);

            // Mark voxels in the inspection sensor's frustum as viewed
            FrustumAngles frustum_angles = octomap_manager_->getCameraFOV(camera_intrinsics_);
            octomap_manager_->markFrustumAsViewed(T_G_inspection_sensor, frustum_angles.horizontal, frustum_angles.vertical, camera_intrinsics_.max_range);
        }
        
        received_first_cloud_ = true;
        
        // Publish visualization
        publishOctomapMarkers();
        
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF exception: %s", ex.what());
    }
}

void NBVPlannerNode::planningTimerCallback()
{
    if (!received_first_cloud_) {
        return;
    }
    
    try {
        // Get current robot pose
        geometry_msgs::msg::TransformStamped transform;
        transform = tf2_buffer_->lookupTransform(
            map_frame_, robot_frame_, tf2::TimePointZero);
        
        current_pose_.position.x = transform.transform.translation.x;
        current_pose_.position.y = transform.transform.translation.y;
        current_pose_.position.z = transform.transform.translation.z;
        current_pose_.orientation = transform.transform.rotation;
        
        // Plan next best view
        auto goal = nbv_planner_->planNextBestView(
            octomap_manager_->getOctree(),
            current_pose_);
        
        // Publish goal
        goal_pub_->publish(goal);
        
        publishFrustumMarker();
        for (const auto& inspection_sensor_frame : inspection_sensor_frames_) {
            publishInspectionFrustumMarker(inspection_sensor_frame);
        }
        // publishCandidateMarkers();
        
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
    }
}

void NBVPlannerNode::publishOctomapMarkers()
{
    // Simple visualization of occupied voxels
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker free_marker;
    visualization_msgs::msg::Marker occupied_marker;
    visualization_msgs::msg::Marker viewed_marker;
    
    free_marker.header.frame_id = map_frame_;
    free_marker.header.stamp = this->now();
    free_marker.ns = "free_cells";
    free_marker.id = 1;
    free_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    free_marker.action = visualization_msgs::msg::Marker::ADD;
    free_marker.scale.x = octomap_manager_->getResolution();
    free_marker.scale.y = octomap_manager_->getResolution();
    free_marker.scale.z = octomap_manager_->getResolution();
    free_marker.color.r = 0.0;
    free_marker.color.g = 1.0;
    free_marker.color.b = 0.0;
    free_marker.color.a = 0.2;

    occupied_marker.header.frame_id = map_frame_;
    occupied_marker.header.stamp = this->now();
    occupied_marker.ns = "occupied_cells";
    occupied_marker.id = 2;
    occupied_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    occupied_marker.action = visualization_msgs::msg::Marker::ADD;
    occupied_marker.scale.x = octomap_manager_->getResolution();
    occupied_marker.scale.y = octomap_manager_->getResolution();
    occupied_marker.scale.z = octomap_manager_->getResolution();
    occupied_marker.color.r = 1.0;
    occupied_marker.color.g = 0.0;
    occupied_marker.color.b = 0.0;
    occupied_marker.color.a = 1.0;

    viewed_marker.header.frame_id = map_frame_;
    viewed_marker.header.stamp = this->now();
    viewed_marker.ns = "viewed_cells";
    viewed_marker.id = 3;
    viewed_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    viewed_marker.action = visualization_msgs::msg::Marker::ADD;
    viewed_marker.scale.x = octomap_manager_->getResolution();
    viewed_marker.scale.y = octomap_manager_->getResolution();
    viewed_marker.scale.z = octomap_manager_->getResolution();
    viewed_marker.color.r = 0.0;
    viewed_marker.color.g = 0.0;
    viewed_marker.color.b = 1.0;
    viewed_marker.color.a = 1.0;
    
    const auto& octree = octomap_manager_->getOctree();
    
    // Iterate through all leaf nodes
    for (auto it = octree.begin_leafs(); it != octree.end_leafs(); ++it) {
        geometry_msgs::msg::Point point;
        point.x = it.getX();
        point.y = it.getY();
        point.z = it.getZ();

        if (octree.isNodeOccupied(*it)) {
            if (octomap_manager_->isVoxelViewed(it.getKey())) {
                viewed_marker.points.push_back(point);
            } else {
                occupied_marker.points.push_back(point);
            }
        } else {
            free_marker.points.push_back(point);
        }
    }
    
    marker_array.markers.push_back(free_marker);
    marker_array.markers.push_back(occupied_marker);
    marker_array.markers.push_back(viewed_marker);
    markers_pub_->publish(marker_array);
}

// Placeholder for candidate poses visualization
void NBVPlannerNode::publishCandidateMarkers()
{
    visualization_msgs::msg::MarkerArray marker_array;
    
    const auto& candidates = nbv_planner_->getCandidates();
    
    for (size_t i = 0; i < candidates.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp = this->now();
        marker.ns = "candidates";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose = candidates[i].pose;
        marker.scale.x = 0.5;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        
        // Color by utility (green = high, red = low)
        double normalized_utility = std::min(candidates[i].utility / 100.0, 1.0);
        marker.color.r = 1.0 - normalized_utility;
        marker.color.g = normalized_utility;
        marker.color.b = 0.0;
        marker.color.a = 0.7;
        
        marker_array.markers.push_back(marker);
    }
    
    candidates_pub_->publish(marker_array);
}

} // namespace nbv_planner

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nbv_planner::NBVPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}