#ifndef NBV_PLANNER_OCTOMAP_MANAGER_HPP
#define NBV_PLANNER_OCTOMAP_MANAGER_HPP

#include <memory>
#include <ufo/map/occupancy_map.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "geometry_msgs/msg/transform.hpp"
#include <rclcpp/rclcpp.hpp>

namespace nbv_planner {

struct UFOMapParameters {
    double resolution = 0.1; // Voxel size in m.
    double sensor_max_range = 5.0;      // Maximum sensor range (m)
    double sensor_min_range = 0.1;      // Minimum sensor range (m)
    double sensor_hfov = 130.0 * M_PI / 180.0; // Horizontal field of view in radians
    double sensor_vfov = 20.0 * M_PI / 180.0;  // Vertical field of view in radians
    // Number of levels for the octree. Has to be [2, 21].
    // This determines the maximum volume that can be represented by the map
    // resolution * 2^(depth_levels) meter in each dimension.
    ufo::map::DepthType depth_levels = 16; // Default: 16
    // If automatic pruning should be enabled/disabled.
    // If disabled it is possible to safely integrate and access data
    // at the same time. Can manually prune the tree.
    bool automatic_pruning = false; // Default: true
    
    // Occupied threshold. Voxels with an occupancy value
    // strictly above this will be classified as occupied.
    double occupied_thres = 0.9; // Default: 0.5
    // Free threshold. Voxels with an occupancy value
    // strictly below this will be classified as free.
    double free_thres = 0.49; // Default: 0.5
    // Voxels with an occupancy value free_thres <= v <= occupied_thres
    // will be classified as unknown.

    // How much the occupancy value of a voxel should be increased when "hit".
    // This value should be (0.5, 1].
    double prob_hit = 0.7; // Default: 0.7
    // How much the occupancy value of a voxel should be decreased when "miss".
    // This value should be [0, 0.5).
    double prob_miss = 0.4; // Default: 0.4

    // Minimum and maximum clamping thresholds. These are used to increased 
    // the amount the can be pruned. A voxel with with an occupancy value 
    // clamping_thres_max < v < clamping_thres_min will be clamped to either 
    // clamping_thres_min or clamping_thres_max.
    double clamping_thres_min = 0.1192; // Default: 0.1192
    double clamping_thres_max = 0.971;  // Default: 0.971

    // The depth at which free space should be cleared.
    // A higher value significantly increases the integration speed
    // for smaller voxel sizes.
    // Will free space at resolution * 2^(integration_depth) voxel size.
    ufo::map::DepthType integration_depth = 1;
};

struct CameraIntrinsics {
    double fx, fy;      // Focal lengths
    double cx, cy;      // Principal point
    int width, height;  // Image dimensions
    double max_range;   // Maximum camera range
};

struct FrustumAngles {
    double horizontal;
    double vertical;
};

// struct MapBounds {
//     octomap::point3d min;
//     octomap::point3d max;
// };

class UFOMapManager {
public:

    /**
     * @brief Constructor
     * @param params UFOMap parameters
     */
    explicit UFOMapManager(UFOMapParameters params = UFOMapParameters());
    
    ~UFOMapManager() = default;

    /**
     * @brief Get read-only access to the map
     */
    const ufo::map::OccupancyMap& getMap() const { return map_; }

    /**
     * @brief Get mutable access to the map
     */
    ufo::map::OccupancyMap& getMapMutable() { return map_; }

    // /**
    //  * @brief Get octree resolution
    //  */
    // double getResolution() const { return octree_->getResolution(); }

    /**
     * @brief Get read-only access to the parameters
     */
    const UFOMapParameters& getParams() const { return params_; }

    void insertPointCloudIntoMap(ufo::map::PointCloud& cloud, const ufo::math::Pose6& T_G_sensor);

    // /**
    //  * @brief Reset the map
    //  */
    // void reset();

    // /**
    //  * @brief Save map to file
    //  */
    // bool saveMap(const std::string& filename);

    // /**
    //  * @brief Load map from file
    //  */
    // bool loadMap(const std::string& filename);

    // /**
    //  * @brief Get map bounds
    //  */
    // MapBounds getMapBounds() const;

    FrustumAngles getCameraFOV(const CameraIntrinsics& intrinsics) const;

    // void markFrustumAsViewed(const Eigen::Isometry3d& T_G_sensor, const double fov_h, const double fov_v, const double max_range);

    // bool isPointViewed(const octomap::point3d& point) const;

    // bool isVoxelViewed(const octomap::OcTreeKey& key) const;

private:
    // void castRay(const octomap::point3d& sensor_origin, const octomap::point3d& point, octomap::KeySet* free_cells, octomap::KeySet* occupied_cells);

    // void updateOccupancy(octomap::KeySet* free_cells, octomap::KeySet* occupied_cells);

    UFOMapParameters params_;
    ufo::map::OccupancyMap map_;

    // octomap::KeyRay key_ray_;
    // octomap::KeyRay key_ray_inspection_;

    // Set of voxels viewed by the inspection sensor in the two sensors configuration (e.g. sonar + camera)
    // octomap::KeySet viewed_voxels_;
};

} // namespace nbv_planner

#endif