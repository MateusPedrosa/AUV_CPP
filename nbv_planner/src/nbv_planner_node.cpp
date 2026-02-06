#include "nbv_planner/nbv_planner_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>

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
    this->declare_parameter("sensor_range", 5.0);
    this->declare_parameter("sensor_min_range", 0.1);
    this->declare_parameter("max_free_space", 0.0);
    this->declare_parameter("min_height_free_space", 0.0);
    this->declare_parameter("sensor_hfov", 130.0 * M_PI / 180.0);
    this->declare_parameter("sensor_vfov", 20.0 * M_PI / 180.0); // 60 degrees
    
    // Get parameters
    map_frame_ = this->get_parameter("map_frame").as_string();
    robot_frame_ = this->get_parameter("robot_frame").as_string();
    planning_frequency_ = this->get_parameter("planning_frequency").as_double();
    
    // Configure octomap parameters
    OctomapParameters octomap_params;
    octomap_params.resolution = this->get_parameter("octree_resolution").as_double();
    octomap_params.sensor_max_range = this->get_parameter("sensor_range").as_double();
    octomap_params.sensor_min_range = this->get_parameter("sensor_min_range").as_double();
    octomap_params.max_free_space = this->get_parameter("max_free_space").as_double();
    octomap_params.min_height_free_space = this->get_parameter("min_height_free_space").as_double();
    octomap_params.sensor_hfov = this->get_parameter("sensor_hfov").as_double();
    octomap_params.sensor_vfov = this->get_parameter("sensor_vfov").as_double();
    
    // Initialize components
    octomap_manager_ = std::make_unique<OctomapManager>(octomap_params);
    nbv_planner_ = std::make_unique<NBVPlanner>(this->get_logger());
    
    // Configure planner
    nbv_planner_->setParameters(
        octomap_params.sensor_max_range, M_PI/2, M_PI/3, 8, 3);
    
    // Initialize TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    
    // Create subscribers
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_in", qos,
        std::bind(&NBVPlannerNode::pointCloudCallback, this, std::placeholders::_1));
    
    // Create publishers
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "nbv_goal", 10);
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "octomap_markers", 10);
    candidates_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "candidate_markers", 10);
    frustum_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("sensor_frustum", 10);
    
    // Create timer for planning
    auto period = std::chrono::duration<double>(1.0 / planning_frequency_);
    planning_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&NBVPlannerNode::planningTimerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "NBV Planner Node initialized");
    RCLCPP_INFO(this->get_logger(), "  Resolution: %.3f m", octomap_params.resolution);
    RCLCPP_INFO(this->get_logger(), "  Sensor range: [%.2f, %.2f] m", 
                octomap_params.sensor_min_range, octomap_params.sensor_max_range);
}

void NBVPlannerNode::publishFrustumMarker() {
    const auto& params = octomap_manager_->getParams();
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "sonar_link"; // Use the actual sensor frame
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

void NBVPlannerNode::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    try {
        // Get transform from sensor to map frame
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped = tf_buffer_->lookupTransform(
            map_frame_,
            msg->header.frame_id,
            msg->header.stamp,
            rclcpp::Duration::from_seconds(0.1));
        
        // Convert to Eigen transform
        Eigen::Isometry3d T_G_sensor = tf2::transformToEigen(transform_stamped);
        
        // Convert ROS PointCloud2 to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);
        
        // Update octree using batch ray casting
        octomap_manager_->insertPointCloudIntoMap(cloud, T_G_sensor);
        
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
        transform = tf_buffer_->lookupTransform(
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
        
        // Publish candidate visualization
        publishFrustumMarker();
        publishCandidateMarkers();
        
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
    }
}

void NBVPlannerNode::publishOctomapMarkers()
{
    // Simple visualization of occupied voxels
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    
    marker.header.frame_id = map_frame_;
    marker.header.stamp = this->now();
    marker.ns = "occupied_cells";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = octomap_manager_->getResolution();
    marker.scale.y = octomap_manager_->getResolution();
    marker.scale.z = octomap_manager_->getResolution();
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.5;
    
    const auto& octree = octomap_manager_->getOctree();
    
    // Iterate through all leaf nodes
    for (auto it = octree.begin_leafs(); it != octree.end_leafs(); ++it) {
        if (octree.isNodeOccupied(*it)) {
            geometry_msgs::msg::Point point;
            point.x = it.getX();
            point.y = it.getY();
            point.z = it.getZ();
            marker.points.push_back(point);
        }
    }
    
    marker_array.markers.push_back(marker);
    markers_pub_->publish(marker_array);
}

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