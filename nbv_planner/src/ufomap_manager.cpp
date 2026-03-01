#include "nbv_planner/ufomap_manager.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>

namespace nbv_planner {

ufo::math::Pose6 poseEigenToUFOMap(const Eigen::Isometry3d& transform)
{
    const Eigen::Vector3d& t = transform.translation();
    Eigen::Quaterniond q(transform.rotation());

    return ufo::math::Pose6(
        t.x(), t.y(), t.z(),
        q.w(), q.x(), q.y(), q.z()
    );
}

UFOMapManager::UFOMapManager(UFOMapParameters params)
    : params_(std::move(params)),
      map_(params_.resolution,
           params_.depth_levels,
           params_.automatic_pruning,
           params_.occupied_thres,
           params_.free_thres,
           params_.prob_hit,
           params_.prob_miss,
           params_.clamping_thres_min,
           params_.clamping_thres_max)
{
    RCLCPP_INFO(rclcpp::get_logger("ufomap_manager"), "UFOMapManager created with depth levels: %d", params_.depth_levels);
}

void UFOMapManager::insertPointCloudIntoMap(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const Eigen::Isometry3d& T_G_sensor)
{
    // Maximum range to integrate, in meters.
    // Set to negative value to ignore maximum range.
    double max_range = params_.sensor_max_range;

    // Remove NaN values, if any.
    std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    pcl::removeNaNFromPointCloud(cloud, filtered_cloud, indices);

    ufo::map::PointCloud ufo_cloud;
    ufo_cloud.reserve(filtered_cloud.size());
    for (const auto& pcl_point : filtered_cloud) {
        ufo_cloud.push_back(ufo::map::Point3(pcl_point.x, pcl_point.y, pcl_point.z));
    }

    ufo::math::Pose6 sensor_frame = poseEigenToUFOMap(T_G_sensor);

    // Specify if the point cloud should be transformed in parallel or not.
    bool parallel = true;
    // Transform point cloud to correct frame
    ufo_cloud.transform(sensor_frame, parallel);

    // Integrate point cloud into UFOMap
    map_.insertPointCloudDiscrete(sensor_frame.translation(), ufo_cloud, max_range, params_.integration_depth);
}

// void UFOMapManager::castRay(
//     const octomap::point3d& sensor_origin,
//     const octomap::point3d& point,
//     octomap::KeySet* free_cells,
//     octomap::KeySet* occupied_cells)
// {
//     if (free_cells == nullptr || occupied_cells == nullptr) {
//         return;
//     }

//     if (params_.sensor_max_range < 0.0 ||
//         (point - sensor_origin).norm() <= params_.sensor_max_range) {
//         // Cast a ray to compute all the free cells.
//         key_ray_.reset();
//         if (octree_->computeRayKeys(sensor_origin, point, key_ray_)) {
//             if (params_.max_free_space == 0.0) {
//                 free_cells->insert(key_ray_.begin(), key_ray_.end());
//             } else {
//                 for (const auto& key : key_ray_) {
//                     octomap::point3d voxel_coordinate = octree_->keyToCoord(key);
//                     if ((voxel_coordinate - sensor_origin).norm() < params_.max_free_space ||
//                         voxel_coordinate.z() > (sensor_origin.z() - params_.min_height_free_space)) {
//                         free_cells->insert(key);
//                     }
//                 }
//             }
//         }
//         // Mark endpoing as occupied.
//         octomap::OcTreeKey key;
//         if (octree_->coordToKeyChecked(point, key)) {
//             occupied_cells->insert(key);
//         }
//     } else {
//         // If the ray is longer than the max range, just update free space.
//         octomap::point3d new_end =
//             sensor_origin +
//             (point - sensor_origin).normalized() * params_.sensor_max_range;
//         key_ray_.reset();
//         if (octree_->computeRayKeys(sensor_origin, new_end, key_ray_)) {
//             if (params_.max_free_space == 0.0) {
//                 free_cells->insert(key_ray_.begin(), key_ray_.end());
//             } else {
//                 for (const auto& key : key_ray_) {
//                     octomap::point3d voxel_coordinate = octree_->keyToCoord(key);
//                     if ((voxel_coordinate - sensor_origin).norm() < params_.max_free_space ||
//                         voxel_coordinate.z() > (sensor_origin.z() - params_.min_height_free_space)) {
//                         free_cells->insert(key);
//                     }
//                 }
//             }
//         }
//     }
// }

// void UFOMapManager::castRayInspection(
//     const octomap::point3d& sensor_origin,
//     const octomap::point3d& point,
//     octomap::KeySet* viewed_cells)
// {
//     if (viewed_cells == nullptr) {
//         return;
//     }

//     // Determine the ray endpoint (clamp to max range if needed)
//     octomap::point3d ray_end = point;
//     double point_distance = (point - sensor_origin).norm();
    
//     if (params_.sensor_max_range > 0.0 && point_distance > params_.sensor_max_range) {
//         ray_end = sensor_origin + (point - sensor_origin).normalized() * params_.sensor_max_range;
//     }

//     // Cast a ray to compute all the free cells.
//     key_ray_inspection_.reset();
//     if (octree_->computeRayKeys(sensor_origin, ray_end, key_ray_inspection_)) {
//         if (params_.max_free_space == 0.0) {
//             viewed_cells->insert(key_ray_inspection_.begin(), key_ray_inspection_.end());
//         } else {
//             for (const auto& key : key_ray_inspection_) {
//                 octomap::point3d voxel_coordinate = octree_->keyToCoord(key);
//                 double voxel_distance = (voxel_coordinate - sensor_origin).norm();
//                 if (voxel_distance < params_.max_free_space ||
//                     voxel_coordinate.z() > (sensor_origin.z() - params_.min_height_free_space)) {
//                     viewed_cells->insert(key);
//                 }
//             }
//         }
//     }
// }

FrustumAngles UFOMapManager::getCameraFOV(const CameraIntrinsics& intrinsics) const {
    double fov_h = 2 * std::atan(intrinsics.width / (2.0 * intrinsics.fx));   // Horizontal
    double fov_v = 2 * std::atan(intrinsics.height / (2.0 * intrinsics.fy));  // Vertical
    return {fov_h, fov_v};
}

// void UFOMapManager::markFrustumAsViewed(
//     const Eigen::Isometry3d& T_G_sensor,
//     const double fov_h,
//     const double fov_v,
//     const double max_range)
// {
//     const octomap::point3d sensor_origin = pointEigenToOctomap(T_G_sensor.translation());
    
//     double half_fov_h = fov_h / 2.0;   // Horizontal
//     double half_fov_v = fov_v / 2.0;  // Vertical
    
//     // Sample rays across the frustum
//     const int horizontal_rays = 40;
//     const int vertical_rays = 30;
    
//     for (int h = 0; h < horizontal_rays; ++h) {
//         for (int v = 0; v < vertical_rays; ++v) {
//             // Angular position within frustum
//             double angle_h = -half_fov_h + (2.0 * half_fov_h * h) / (horizontal_rays - 1);
//             double angle_v = -half_fov_v + (2.0 * half_fov_v * v) / (vertical_rays - 1);
            
//             // Ray direction in sensor frame
//             double horizontal_offset = -std::tan(angle_h); // -Y is right
//             double vertical_offset = std::tan(angle_v);    // +Z is up
//             Eigen::Vector3d ray(1.0, horizontal_offset, vertical_offset);
//             ray.normalize();
            
//             // Transform to world frame
//             Eigen::Vector3d ray_world = T_G_sensor.rotation() * ray;
//             Eigen::Vector3d endpoint_world = T_G_sensor.translation() + ray_world * max_range;
//             octomap::point3d endpoint_octo = pointEigenToOctomap(endpoint_world);
//             octomap::point3d ray_world_octo = pointEigenToOctomap(ray_world);
            
//             // Cast ray and find where it hits occupied/unknown space
//             octomap::point3d ray_end;
//             bool hit_obstacle = octree_->castRay(sensor_origin, ray_world_octo, ray_end, false, max_range);
            
//             // Mark all voxels along the ray as viewed (up to obstacle or max range)
//             octomap::point3d actual_end = hit_obstacle ? ray_end : endpoint_octo;
            
//             key_ray_.reset();
//             if (octree_->computeRayKeys(sensor_origin, actual_end, key_ray_)) {
//                 viewed_voxels_.insert(key_ray_.begin(), key_ray_.end());
//             }
//         }
//     }
// }

// void UFOMapManager::reset() {
//     double resolution = octree_->getResolution();
//     octree_ = std::make_unique<octomap::OcTree>(resolution);
// }

// bool UFOMapManager::saveMap(const std::string& filename) {
//     return octree_->write(filename);
// }

// bool UFOMapManager::loadMap(const std::string& filename) {
//     auto loaded_tree = std::unique_ptr<octomap::OcTree>(
//         dynamic_cast<octomap::OcTree*>(octomap::OcTree::read(filename)));
    
//     if (loaded_tree) {
//         octree_ = std::move(loaded_tree);
//         return true;
//     }
//     return false;
// }

// MapBounds UFOMapManager::getMapBounds() const {
//     double minX, minY, minZ, maxX, maxY, maxZ;
//     octree_->getMetricMin(minX, minY, minZ);
//     octree_->getMetricMax(maxX, maxY, maxZ);
    
//     return { 
//         octomap::point3d((float)minX, (float)minY, (float)minZ), 
//         octomap::point3d((float)maxX, (float)maxY, (float)maxZ) 
//     };
// }

// bool UFOMapManager::isVoxelViewed(const octomap::OcTreeKey& key) const
// {
//     return viewed_voxels_.find(key) != viewed_voxels_.end();
// }

// bool UFOMapManager::isPointViewed(const octomap::point3d& point) const
// {
//     octomap::OcTreeKey key;
//     if (octree_->coordToKeyChecked(point, key)) {
//         return isVoxelViewed(key);
//     }
//     return false;
// }

} // namespace nbv_planner