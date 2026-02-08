#ifndef NBV_PLANNER_OCTOMAP_MANAGER_HPP
#define NBV_PLANNER_OCTOMAP_MANAGER_HPP

#include <memory>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace nbv_planner {

struct OctomapParameters {
    double sensor_max_range = 5.0;      // Maximum sensor range (m)
    double sensor_min_range = 0.1;      // Minimum sensor range (m)
    double sensor_hfov = 130.0 * M_PI / 180.0; // Horizontal field of view in radians
    double sensor_vfov = 20.0 * M_PI / 180.0;  // Vertical field of view in radians
    double max_free_space = 0.0;        // Max distance to mark as free (0 = unlimited)
    double min_height_free_space = 0.0; // Min height for free space marking
    double resolution = 0.1;            // Octree resolution
    
    // Probability parameters
    double prob_hit = 0.7;
    double prob_miss = 0.4;
    double clamping_thres_min = 0.12;
    double clamping_thres_max = 0.97;
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

class OctomapManager {
public:
    /**
     * @brief Constructor
     * @param params Octomap parameters
     */
    explicit OctomapManager(const OctomapParameters& params = OctomapParameters());
    
    ~OctomapManager() = default;

    /**
     * @brief Get read-only access to the octree
     */
    const octomap::OcTree& getOctree() const { return *octree_; }

    /**
     * @brief Get mutable access to the octree (use carefully)
     */
    octomap::OcTree& getOctreeMutable() { return *octree_; }

    /**
     * @brief Get octree resolution
     */
    double getResolution() const { return octree_->getResolution(); }

    /**
     * @brief Get read-only access to the parameters
     */
    const OctomapParameters& getParams() const { return params_; }

    void insertPointCloudIntoMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Eigen::Isometry3d& T_G_sensor);

    /**
     * @brief Reset the map
     */
    void reset();

    /**
     * @brief Save map to file
     */
    bool saveMap(const std::string& filename);

    /**
     * @brief Load map from file
     */
    bool loadMap(const std::string& filename);

    /**
     * @brief Get map bounds
     */
    void getMapBounds(octomap::point3d& min, octomap::point3d& max) const;

    FrustumAngles getCameraFOV(const CameraIntrinsics& intrinsics) const;

    void markFrustumAsViewed(const Eigen::Isometry3d& T_G_sensor, const double fov_h, const double fov_v, const double max_range);

    bool isPointViewed(const octomap::point3d& point) const;

    bool isVoxelViewed(const octomap::OcTreeKey& key) const;

private:
    void castRay(const octomap::point3d& sensor_origin, const octomap::point3d& point, octomap::KeySet* free_cells, octomap::KeySet* occupied_cells);

    void updateOccupancy(octomap::KeySet* free_cells, octomap::KeySet* occupied_cells);

    std::unique_ptr<octomap::OcTree> octree_;
    OctomapParameters params_;

    octomap::KeyRay key_ray_;
    // octomap::KeyRay key_ray_inspection_;

    // Set of voxels viewed by the inspection sensor in the two sensors configuration (e.g. sonar + camera)
    octomap::KeySet viewed_voxels_;
};

} // namespace nbv_planner

#endif