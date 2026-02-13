#ifndef NBV_PLANNER_VOXBLOX_MANAGER_HPP
#define NBV_PLANNER_VOXBLOX_MANAGER_HPP

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/layer_io.h>
#include <unordered_set>

namespace nbv_planner {

// Voxblox parameters structure
struct VoxbloxParameters {
    // TSDF map configuration
    float voxel_size = 0.1f;                    // Size of each voxel in meters
    int voxels_per_side = 16;                   // Number of voxels per block side
    
    // TSDF integrator configuration
    float truncation_distance = 0.4f;           // Truncation distance for TSDF
    float max_ray_length = 5.0f;                // Maximum ray length in meters
    float min_ray_length = 0.1f;                // Minimum ray length in meters
    float max_weight = 10000.0f;                // Maximum weight for voxel integration
    bool voxel_carving_enabled = true;          // Enable voxel carving
    bool use_const_weight = false;              // Use constant weight
    bool allow_clear = true;                    // Allow clearing of voxels
    float start_voxel_subsampling_factor = 2.0f; // Subsampling factor for voxel updates
    int max_consecutive_ray_collisions = 2;     // Max consecutive ray collisions
    int clear_checks_every_n_frames = 1;        // Clear checks frequency
    
    // Sensor configuration
    double sensor_hfov = 130.0 * M_PI / 180.0;  // Horizontal field of view
    double sensor_vfov = 20.0 * M_PI / 180.0;   // Vertical field of view
    
    // Occupancy checking
    float surface_distance_threshold = 0.1f;     // Distance threshold for occupancy
};

// Hash function for voxblox::GlobalIndex
struct GlobalIndexHash {
    std::size_t operator()(const voxblox::GlobalIndex& idx) const {
        std::size_t h1 = std::hash<voxblox::IndexElement>{}(idx.x());
        std::size_t h2 = std::hash<voxblox::IndexElement>{}(idx.y());
        std::size_t h3 = std::hash<voxblox::IndexElement>{}(idx.z());
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

class VoxbloxManager {
    
public:
    explicit VoxbloxManager(const VoxbloxParameters& params);

    // Camera intrinsics structure
    struct CameraIntrinsics {
        double fx;
        double fy;
        double cx;
        double cy;
        int width;
        int height;
        double max_range;
    };

    // Frustum angles structure
    struct FrustumAngles {
        double horizontal;
        double vertical;
    };


    // Map bounds structure
    struct MapBounds {
        voxblox::Point min;
        voxblox::Point max;
    };
    
    /**
     * @brief Insert a point cloud into the TSDF map
     * @param cloud Input point cloud in sensor frame
     * @param T_G_sensor Transform from sensor to global/map frame
     */
    void insertPointCloudIntoMap(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const Eigen::Isometry3d& T_G_sensor);
    
    /**
     * @brief Mark voxels in a camera frustum as viewed
     * @param T_G_sensor Transform from sensor to global frame
     * @param fov_h Horizontal field of view in radians
     * @param fov_v Vertical field of view in radians
     * @param max_range Maximum sensor range
     */
    void markFrustumAsViewed(
        const Eigen::Isometry3d& T_G_sensor,
        const double fov_h,
        const double fov_v,
        const double max_range);
    
    /**
     * @brief Calculate camera field of view from intrinsics
     * @param intrinsics Camera intrinsic parameters
     * @return Horizontal and vertical FOV angles
     */
    FrustumAngles getCameraFOV(const CameraIntrinsics& intrinsics) const;
    
    /**
     * @brief Check if a point is occupied
     * @param point Point to check in global frame
     * @return True if occupied, false otherwise
     */
    bool isOccupied(const voxblox::Point& point) const;
    
    /**
     * @brief Check if a point is known (observed)
     * @param point Point to check in global frame
     * @return True if known, false if unknown
     */
    bool isKnown(const voxblox::Point& point) const;
    
    /**
     * @brief Get the TSDF distance at a point
     * @param point Point to query
     * @return TSDF distance value
     */
    float getDistance(const voxblox::Point& point) const;
    
    /**
     * @brief Reset the map
     */
    void reset();
    
    /**
     * @brief Save map to file
     * @param filename Path to save file
     * @return True if successful
     */
    bool saveMap(const std::string& filename);
    
    /**
     * @brief Load map from file
     * @param filename Path to load file
     * @return True if successful
     */
    bool loadMap(const std::string& filename);
    
    /**
     * @brief Get the bounds of the map
     * @return Min and max coordinates of the map
     */
    MapBounds getMapBounds() const;
    
    /**
     * @brief Check if a voxel has been viewed by inspection sensor
     * @param index Global voxel index
     * @return True if viewed
     */
    bool isVoxelViewed(const voxblox::GlobalIndex& index) const;
    
    /**
     * @brief Check if a point has been viewed by inspection sensor
     * @param point Point in global frame
     * @return True if viewed
     */
    bool isPointViewed(const voxblox::Point& point) const;
    
    /**
     * @brief Get the TSDF layer
     * @return Pointer to TSDF layer
     */
    voxblox::Layer<voxblox::TsdfVoxel>* getTsdfLayer() const;
    
    /**
     * @brief Get the TSDF map
     * @return Shared pointer to TSDF map
     */
    std::shared_ptr<voxblox::TsdfMap> getTsdfMap() const;
    
    /**
     * @brief Get voxel size
     * @return Voxel size in meters
     */
    float getVoxelSize() const { return params_.voxel_size; }
    
    /**
     * @brief Get voxels per side
     * @return Number of voxels per block side
     */
    int getVoxelsPerSide() const { return params_.voxels_per_side; }
    
    /**
     * @brief Get parameters
     * @return Reference to parameters
     */
    const VoxbloxParameters& getParams() const { return params_; }

private:
    std::shared_ptr<voxblox::TsdfMap> tsdf_map_;
    std::shared_ptr<voxblox::TsdfIntegratorBase> tsdf_integrator_;
    VoxbloxParameters params_;
    
    // Set of voxels that have been viewed by inspection sensors
    std::unordered_set<voxblox::GlobalIndex, GlobalIndexHash> viewed_voxels_;
};

} // namespace nbv_planner

#endif // NBV_PLANNER_VOXBLOX_MANAGER_HPP