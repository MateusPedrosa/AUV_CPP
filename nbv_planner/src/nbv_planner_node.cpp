#include "nbv_planner/nbv_planner_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
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
    // this->declare_parameter("planning_frequency", 2.0);
    this->declare_parameter("exploration_sensor_max_range", 40.0);
    this->declare_parameter("exploration_sensor_min_range", 0.1);
    this->declare_parameter("exploration_sensor_hfov", 130.0 * M_PI / 180.0);
    this->declare_parameter("exploration_sensor_vfov", 20.0 * M_PI / 180.0); // 60 degrees
    this->declare_parameter("octree_depth_levels", 16);
    this->declare_parameter("octree_automatic_pruning", false);
    this->declare_parameter("octree_occupied_thres", 0.9);
    this->declare_parameter("octree_free_thres", 0.49);
    this->declare_parameter("octree_prob_hit", 0.7);
    this->declare_parameter("octree_prob_miss", 0.4);
    this->declare_parameter("octree_clamping_thres_min", 0.1192);
    this->declare_parameter("octree_clamping_thres_max", 0.971);
    this->declare_parameter("octree_integration_depth", 1);
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
    // planning_frequency_ = this->get_parameter("planning_frequency").as_double();

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

    // Configure UFOMap parameters
    UFOMapParameters ufomap_params;
    ufomap_params.resolution = this->get_parameter("octree_resolution").as_double();
    ufomap_params.sensor_max_range = this->get_parameter("exploration_sensor_max_range").as_double();
    ufomap_params.sensor_min_range = this->get_parameter("exploration_sensor_min_range").as_double();
    ufomap_params.sensor_hfov = this->get_parameter("exploration_sensor_hfov").as_double();
    ufomap_params.sensor_vfov = this->get_parameter("exploration_sensor_vfov").as_double();
    ufomap_params.depth_levels = static_cast<ufo::map::DepthType>(this->get_parameter("octree_depth_levels").as_int());
    ufomap_params.automatic_pruning = this->get_parameter("octree_automatic_pruning").as_bool();
    ufomap_params.occupied_thres = this->get_parameter("octree_occupied_thres").as_double();
    ufomap_params.free_thres = this->get_parameter("octree_free_thres").as_double();
    ufomap_params.prob_hit = this->get_parameter("octree_prob_hit").as_double();
    ufomap_params.prob_miss = this->get_parameter("octree_prob_miss").as_double();
    ufomap_params.clamping_thres_min = this->get_parameter("octree_clamping_thres_min").as_double();
    ufomap_params.clamping_thres_max = this->get_parameter("octree_clamping_thres_max").as_double();
    ufomap_params.integration_depth = static_cast<ufo::map::DepthType>(this->get_parameter("octree_integration_depth").as_int());
    
    // Initialize components
    ufomap_manager_ = std::make_unique<UFOMapManager>(ufomap_params);
    // nbv_planner_ = std::make_unique<NBVPlanner>(this->get_logger());

    // // Initialize path planner action server with shared ufomap_manager
    // path_planner_ = std::make_unique<SimplePlanner>(this, ufomap_manager_);
    
    // Configure planner (placeholder)
    // nbv_planner_->setParameters(ufomap_params.sensor_max_range, M_PI/2, M_PI/3, 8, 3);
    
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
    
    // Setup the subscriber
    point_cloud_sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());

    // Setup the filter to wait for the transform between the cloud's frame and map_frame
    tf_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        point_cloud_sub_,
        *tf2_buffer_,
        map_frame_,      // The frame we need the transform TO
        10,              // Queue size
        this->get_node_logging_interface(),
        this->get_node_clock_interface(),
        std::chrono::milliseconds(100) // How long to wait for the TF to arrive
    );

    // Register the callback
    tf_filter_->registerCallback(&NBVPlannerNode::pointCloudCallback, this);
    
    // Create publishers
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "nbv_goal", 10);
    map_pub_ = this->create_publisher<ufomap_msgs::msg::UFOMapStamped>("ufomap", 10);
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "ufomap_markers", 10);
    candidates_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "candidate_markers", 10);
    frustum_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("exploration_sensor_frustum", 10);
    inspection_frustum_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("inspection_sensor_frustum", 10);
    
    // // Create timer for planning
    // auto period = std::chrono::duration<double>(1.0 / planning_frequency_);
    // planning_timer_ = this->create_wall_timer(
    //     std::chrono::duration_cast<std::chrono::milliseconds>(period),
    //     std::bind(&NBVPlannerNode::planningTimerCallback, this)
    // );
    
    RCLCPP_INFO(this->get_logger(), "NBV Planner Node initialized");
    RCLCPP_INFO(this->get_logger(), "  Resolution: %.3f m", ufomap_params.resolution);
    RCLCPP_INFO(this->get_logger(), "  Exploration sensor range: [%.2f, %.2f] m", 
                ufomap_params.sensor_min_range, ufomap_params.sensor_max_range);
}

// Publish frustum as an arc-shaped solid volume
void NBVPlannerNode::publishFrustumMarker() {
    const auto& params = ufomap_manager_->getParams();

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = exploration_sensor_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.frame_locked = true;
    marker.ns = "frustum_solid";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.3;

    const float r_min  = params.sensor_min_range;
    const float r_max  = params.sensor_max_range;
    const float h_fov  = params.sensor_hfov;
    const float v_fov  = params.sensor_vfov;

    const int H_SEGS = 8;
    const int V_SEGS = 4;

    const float h_half = h_fov / 2.0f;
    const float v_half = v_fov / 2.0f;

    // Point on the spherical surface at a given range and grid index
    auto arcPoint = [&](float range, int i, int j) -> geometry_msgs::msg::Point {
        float az = -h_half + (float)i / H_SEGS * h_fov;
        float el = -v_half + (float)j / V_SEGS * v_fov;
        geometry_msgs::msg::Point p;
        p.x = range * std::cos(el) * std::cos(az);
        p.y = range * std::cos(el) * std::sin(az);
        p.z = range * std::sin(el);
        return p;
    };

    auto& pts = marker.points;

    // ----------------------------------------------------------------
    // CURVED FAR SURFACE (r_max)
    // ----------------------------------------------------------------
    for (int i = 0; i < H_SEGS; ++i) {
        for (int j = 0; j < V_SEGS; ++j) {
            auto p00 = arcPoint(r_max, i,     j    );
            auto p10 = arcPoint(r_max, i + 1, j    );
            auto p01 = arcPoint(r_max, i,     j + 1);
            auto p11 = arcPoint(r_max, i + 1, j + 1);

            pts.push_back(p00); pts.push_back(p10); pts.push_back(p11);
            pts.push_back(p00); pts.push_back(p11); pts.push_back(p01);
        }
    }

    // ----------------------------------------------------------------
    // CURVED NEAR SURFACE (r_min) — winding flipped to face inward
    // ----------------------------------------------------------------
    for (int i = 0; i < H_SEGS; ++i) {
        for (int j = 0; j < V_SEGS; ++j) {
            auto p00 = arcPoint(r_min, i,     j    );
            auto p10 = arcPoint(r_min, i + 1, j    );
            auto p01 = arcPoint(r_min, i,     j + 1);
            auto p11 = arcPoint(r_min, i + 1, j + 1);

            pts.push_back(p00); pts.push_back(p11); pts.push_back(p10);
            pts.push_back(p00); pts.push_back(p01); pts.push_back(p11);
        }
    }

    // ----------------------------------------------------------------
    // SIDE FACES — quad strips connecting r_min arc to r_max arc
    //    Each quad = two triangles bridging the inner and outer shells
    // ----------------------------------------------------------------

    // Bottom edge (j = 0)
    for (int i = 0; i < H_SEGS; ++i) {
        pts.push_back(arcPoint(r_min, i,     0));
        pts.push_back(arcPoint(r_max, i,     0));
        pts.push_back(arcPoint(r_max, i + 1, 0));

        pts.push_back(arcPoint(r_min, i,     0));
        pts.push_back(arcPoint(r_max, i + 1, 0));
        pts.push_back(arcPoint(r_min, i + 1, 0));
    }

    // Top edge (j = V_SEGS)
    for (int i = 0; i < H_SEGS; ++i) {
        pts.push_back(arcPoint(r_min, i + 1, V_SEGS));
        pts.push_back(arcPoint(r_max, i + 1, V_SEGS));
        pts.push_back(arcPoint(r_max, i,     V_SEGS));

        pts.push_back(arcPoint(r_min, i + 1, V_SEGS));
        pts.push_back(arcPoint(r_max, i,     V_SEGS));
        pts.push_back(arcPoint(r_min, i,     V_SEGS));
    }

    // Left edge (i = 0)
    for (int j = 0; j < V_SEGS; ++j) {
        pts.push_back(arcPoint(r_min, 0, j + 1));
        pts.push_back(arcPoint(r_max, 0, j + 1));
        pts.push_back(arcPoint(r_max, 0, j    ));

        pts.push_back(arcPoint(r_min, 0, j + 1));
        pts.push_back(arcPoint(r_max, 0, j    ));
        pts.push_back(arcPoint(r_min, 0, j    ));
    }

    // Right edge (i = H_SEGS)
    for (int j = 0; j < V_SEGS; ++j) {
        pts.push_back(arcPoint(r_min, H_SEGS, j    ));
        pts.push_back(arcPoint(r_max, H_SEGS, j    ));
        pts.push_back(arcPoint(r_max, H_SEGS, j + 1));

        pts.push_back(arcPoint(r_min, H_SEGS, j    ));
        pts.push_back(arcPoint(r_max, H_SEGS, j + 1));
        pts.push_back(arcPoint(r_min, H_SEGS, j + 1));
    }

    frustum_pub_->publish(marker);
}

// Publish frustum as a volume
// void NBVPlannerNode::publishFrustumMarker() {
//     const auto& params = octomap_manager_->getParams();
//     visualization_msgs::msg::Marker marker;
//     marker.header.frame_id = exploration_sensor_frame_;
//     marker.header.stamp = rclcpp::Time(0);
//     marker.frame_locked = true;
//     marker.ns = "frustum_solid";
//     marker.id = 0;
    
//     // Change to TRIANGLE_LIST for solid volume
//     marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
//     marker.action = visualization_msgs::msg::Marker::ADD;
    
//     // Scale for TRIANGLE_LIST is 1.0 (it doesn't affect thickness)
//     marker.scale.x = 1.0;
//     marker.scale.y = 1.0;
//     marker.scale.z = 1.0;

//     float range = params.sensor_max_range;
//     float h_half = range * tan(params.sensor_hfov / 2.0);
//     float v_half = range * tan(params.sensor_vfov / 2.0);

//     geometry_msgs::msg::Point p0, p1, p2, p3, p4;
//     p0.x = 0; p0.y = 0; p0.z = 0;              // Apex (Origin)
//     p1.x = range; p1.y =  h_half; p1.z =  v_half; // Top Left (Far)
//     p2.x = range; p2.y = -h_half; p2.z =  v_half; // Top Right (Far)
//     p3.x = range; p3.y = -h_half; p3.z = -v_half; // Bottom Right (Far)
//     p4.x = range; p4.y =  h_half; p4.z = -v_half; // Bottom Left (Far)

//     // Define the 6 triangles (3 points each)
//     marker.points = {
//         // Top Face
//         p0, p1, p2,
//         // Right Face
//         p0, p2, p3,
//         // Bottom Face
//         p0, p3, p4,
//         // Left Face
//         p0, p4, p1,
//         // Far Plane (Rectangle made of two triangles)
//         p1, p3, p2,
//         p1, p4, p3
//     };

//     // Semi-transparent Green
//     marker.color.r = 0.0;
//     marker.color.g = 1.0;
//     marker.color.b = 0.0;
//     marker.color.a = 0.3; // Alpha < 1.0 makes it transparent

//     frustum_pub_->publish(marker);
// }

// Publish frustum as a wireframe
// void NBVPlannerNode::publishFrustumMarker() {
//     const auto& params = octomap_manager_->getParams();
//     visualization_msgs::msg::Marker marker;
//     marker.header.frame_id = exploration_sensor_frame_; //TODO: get frame from params
//     marker.header.stamp = rclcpp::Time(0);
//     marker.frame_locked = true;
//     marker.ns = "frustum";
//     marker.id = 0;
//     marker.type = visualization_msgs::msg::Marker::LINE_LIST;
//     marker.action = visualization_msgs::msg::Marker::ADD;
//     marker.scale.x = 0.02; // Line thickness

//     // Calculate frustum corners based on your planner params
//     float range = params.sensor_max_range;
//     float h_half = range * tan(params.sensor_hfov / 2.0);
//     float v_half = range * tan(params.sensor_vfov / 2.0);

//     // Define the 5 points: Origin (0,0,0) and 4 corners of the far plane
//     geometry_msgs::msg::Point p0, p1, p2, p3, p4;
//     p0.x = 0; p0.y = 0; p0.z = 0;
//     p1.x = range; p1.y = h_half; p1.z = v_half;
//     p2.x = range; p2.y = -h_half; p2.z = v_half;
//     p3.x = range; p3.y = -h_half; p3.z = -v_half;
//     p4.x = range; p4.y = h_half; p4.z = -v_half;

//     // Add lines from origin to corners and connecting corners...
//     marker.points = {p0, p1, p0, p2, p0, p3, p0, p4, p1, p2, p2, p3, p3, p4, p4, p1};
//     marker.color.r = 0.0;
//     marker.color.g = 1.0;
//     marker.color.b = 0.0;
//     marker.color.a = 1.0;

//     frustum_pub_->publish(marker);
// }

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

    FrustumAngles frustum_angles = ufomap_manager_->getCameraFOV(camera_intrinsics_);
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

        ufo::math::Pose6 sensor_transform;
        sensor_transform = ufomap_ros::rosToUfo(t_exploration_sensor.transform);

        ufo::map::PointCloud cloud;
        ufomap_ros::rosToUfo(*msg, cloud);
        
        ufomap_manager_->insertPointCloudIntoMap(cloud, sensor_transform);

        // for (const auto& inspection_sensor_frame : inspection_sensor_frames_) {
        //     // Get transform from inspection sensor to map frame
        //     geometry_msgs::msg::TransformStamped t_inspection_sensor;
        //     t_inspection_sensor = tf2_buffer_->lookupTransform(
        //         map_frame_,
        //         inspection_sensor_frame,
        //         msg->header.stamp);
        //     // Convert to Eigen transform
        //     Eigen::Isometry3d T_G_inspection_sensor = tf2::transformToEigen(t_inspection_sensor);

        //     // Mark voxels in the inspection sensor's frustum as viewed
        //     FrustumAngles frustum_angles = ufomap_manager_->getCameraFOV(camera_intrinsics_);
        //     ufomap_manager_->markFrustumAsViewed(T_G_inspection_sensor, frustum_angles.horizontal, frustum_angles.vertical, camera_intrinsics_.max_range);
        // }
        
        received_first_cloud_ = true;

        // Publish visualization
        // publishUFOMapMarkers();
        publishUFOMap();
        
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF exception: %s", ex.what());
    }
}

// void NBVPlannerNode::planningTimerCallback()
// {
//     if (!received_first_cloud_) {
//         return;
//     }

//     try {
//         // Get current robot pose
//         geometry_msgs::msg::TransformStamped transform;
//         transform = tf2_buffer_->lookupTransform(
//             map_frame_, robot_frame_, tf2::TimePointZero);
        
//         current_pose_.position.x = transform.transform.translation.x;
//         current_pose_.position.y = transform.transform.translation.y;
//         current_pose_.position.z = transform.transform.translation.z;
//         current_pose_.orientation = transform.transform.rotation;
        
//         // Plan next best view
//         auto goal = nbv_planner_->planNextBestView(
//             octomap_manager_->getOctree(),
//             current_pose_);
        
//         // Publish goal
//         goal_pub_->publish(goal);
        
//         publishFrustumMarker();
//         for (const auto& inspection_sensor_frame : inspection_sensor_frames_) {
//             publishInspectionFrustumMarker(inspection_sensor_frame);
//         }
//         // publishCandidateMarkers();
        
//     } catch (tf2::TransformException& ex) {
//         RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
//     }

//     publishUFOMap();
// }

void NBVPlannerNode::publishUFOMap()
{
    // If the UFOMap should be compressed using LZ4.
    // Good if you are sending the UFOMap between computers.
    bool compress = false;

    // Lowest depth to publish.
    // Higher value means less data to transfer, good in
    // situation where the data rate is low.
    // Many nodes do not require detailed maps as well.
    ufo::map::DepthType pub_depth = 0;

    // Get the bounding box of only the updated region
    ufo::geometry::AABB updated_aabb = ufomap_manager_->getChangeBoundingBox();
    ufo::geometry::BoundingVolume bv;
    bv.add(updated_aabb);

    auto msg = std::make_shared<ufomap_msgs::msg::UFOMapStamped>();
    if (ufomap_msgs::ufoToMsg(ufomap_manager_->getMap(), msg->map, bv, compress, pub_depth)) {
        msg->header.stamp = this->now();
        msg->header.frame_id = map_frame_;
        map_pub_->publish(*msg);

        // Reset the change detection so the next loop only catches new updates
        ufomap_manager_->resetChangeDetection();
    } else {
        RCLCPP_INFO(this->get_logger(), "!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
}

void NBVPlannerNode::publishUFOMapMarkers()
{
    // Simple visualization of occupied voxels
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker free_marker;
    visualization_msgs::msg::Marker occupied_marker;
    visualization_msgs::msg::Marker viewed_marker;
    visualization_msgs::msg::Marker unknown_marker;

    const auto& params = ufomap_manager_->getParams();
    
    free_marker.header.frame_id = map_frame_;
    free_marker.header.stamp = this->now();
    free_marker.ns = "free_cells";
    free_marker.id = 1;
    free_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    free_marker.action = visualization_msgs::msg::Marker::ADD;
    free_marker.scale.x = params.resolution;
    free_marker.scale.y = params.resolution;
    free_marker.scale.z = params.resolution;
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
    occupied_marker.scale.x = params.resolution;
    occupied_marker.scale.y = params.resolution;
    occupied_marker.scale.z = params.resolution;
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
    viewed_marker.scale.x = params.resolution;
    viewed_marker.scale.y = params.resolution;
    viewed_marker.scale.z = params.resolution;
    viewed_marker.color.r = 0.0;
    viewed_marker.color.g = 0.0;
    viewed_marker.color.b = 1.0;
    viewed_marker.color.a = 1.0;

    unknown_marker.header.frame_id = map_frame_;
    unknown_marker.header.stamp = this->now();
    unknown_marker.ns = "unknown_cells";
    unknown_marker.id = 4;
    unknown_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    unknown_marker.action = visualization_msgs::msg::Marker::ADD;
    unknown_marker.scale.x = params.resolution;
    unknown_marker.scale.y = params.resolution;
    unknown_marker.scale.z = params.resolution;
    unknown_marker.color.r = 1.0;
    unknown_marker.color.g = 1.0;
    unknown_marker.color.b = 1.0;
    unknown_marker.color.a = 1.0;
    
    const ufo::map::OccupancyMap& map = ufomap_manager_->getMap();
    
    // Iterate through all leaf nodes
    for (auto it = map.beginLeaves(true, false, false, false, 0), it_end = map.endLeaves(); it != it_end; ++it) {
        geometry_msgs::msg::Point point;
        point.x = it.getX();
        point.y = it.getY();
        point.z = it.getZ();

        // if (octree.isNodeOccupied(*it)) {
        //     if (ufomap_manager_->isVoxelViewed(it.getKey())) {
        //         viewed_marker.points.push_back(point);
        //     } else {
        //         occupied_marker.points.push_back(point);
        //     }
        // } else {
        //     free_marker.points.push_back(point);
        // }
        occupied_marker.points.push_back(point);
    }

    for (auto it = map.beginLeaves(false, true, false, false, 0), it_end = map.endLeaves(); it != it_end; ++it) {
        geometry_msgs::msg::Point point;
        point.x = it.getX();
        point.y = it.getY();
        point.z = it.getZ();

        free_marker.points.push_back(point);
    }

    for (auto it = map.beginLeaves(false, false, true, false, 0), it_end = map.endLeaves(); it != it_end; ++it) {
        geometry_msgs::msg::Point point;
        point.x = it.getX();
        point.y = it.getY();
        point.z = it.getZ();

        unknown_marker.points.push_back(point);
    }
    
    marker_array.markers.push_back(free_marker);
    marker_array.markers.push_back(occupied_marker);
    marker_array.markers.push_back(viewed_marker);
    marker_array.markers.push_back(unknown_marker);
    markers_pub_->publish(marker_array);
}

// // Placeholder for candidate poses visualization
// void NBVPlannerNode::publishCandidateMarkers()
// {
//     visualization_msgs::msg::MarkerArray marker_array;
    
//     const auto& candidates = nbv_planner_->getCandidates();
    
//     for (size_t i = 0; i < candidates.size(); ++i) {
//         visualization_msgs::msg::Marker marker;
//         marker.header.frame_id = map_frame_;
//         marker.header.stamp = this->now();
//         marker.ns = "candidates";
//         marker.id = i;
//         marker.type = visualization_msgs::msg::Marker::ARROW;
//         marker.action = visualization_msgs::msg::Marker::ADD;
        
//         marker.pose = candidates[i].pose;
//         marker.scale.x = 0.5;
//         marker.scale.y = 0.05;
//         marker.scale.z = 0.05;
        
//         // Color by utility (green = high, red = low)
//         double normalized_utility = std::min(candidates[i].utility / 100.0, 1.0);
//         marker.color.r = 1.0 - normalized_utility;
//         marker.color.g = normalized_utility;
//         marker.color.b = 0.0;
//         marker.color.a = 0.7;
        
//         marker_array.markers.push_back(marker);
//     }
    
//     candidates_pub_->publish(marker_array);
// }

} // namespace nbv_planner

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nbv_planner::NBVPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}