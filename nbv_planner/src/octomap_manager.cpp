#include "nbv_planner/octomap_manager.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>

namespace nbv_planner {

octomap::point3d pointEigenToOctomap(const Eigen::Vector3d& point)
{
    return octomap::point3d(point.x(), point.y(), point.z());
}

OctomapManager::OctomapManager(const OctomapParameters& params)
    : octree_(std::make_unique<octomap::OcTree>(params.resolution)),
    params_(params)
{
    // Configure octree probability parameters
    octree_->setProbHit(params_.prob_hit);
    octree_->setProbMiss(params_.prob_miss);
    octree_->setClampingThresMin(params_.clamping_thres_min);
    octree_->setClampingThresMax(params_.clamping_thres_max);
}

void OctomapManager::insertPointCloudIntoMap(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const Eigen::Isometry3d& T_G_sensor)
{

    // Remove NaN values, if any.
    std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::removeNaNFromPointCloud(*cloud, *filtered_cloud, indices);

    // First, rotate the pointcloud into the world frame.
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*filtered_cloud, *transformed_cloud, T_G_sensor.matrix().cast<float>());
    const octomap::point3d p_G_sensor = pointEigenToOctomap(T_G_sensor.translation());

    // Then add all the rays from this pointcloud.
    // We do this as a batch operation - so first get all the keys in a set, then
    // do the update in batch.
    octomap::KeySet free_cells, occupied_cells;
    for (const auto& point : transformed_cloud->points) {
        const octomap::point3d p_G_point(point.x, point.y, point.z);
        // First, check if we've already checked this.
        octomap::OcTreeKey key = octree_->coordToKey(p_G_point);

        if (occupied_cells.find(key) == occupied_cells.end()) {
        // Check if this is within the allowed sensor range.
        castRay(p_G_sensor, p_G_point, &free_cells, &occupied_cells);
        }
    }

    // Apply the new free cells and occupied cells from
    updateOccupancy(&free_cells, &occupied_cells);
}

void OctomapManager::castRay(
    const octomap::point3d& sensor_origin,
    const octomap::point3d& point,
    octomap::KeySet* free_cells,
    octomap::KeySet* occupied_cells)
{
    if (free_cells == nullptr || occupied_cells == nullptr) {
        return;
    }

    if (params_.sensor_max_range < 0.0 ||
        (point - sensor_origin).norm() <= params_.sensor_max_range) {
        // Cast a ray to compute all the free cells.
        key_ray_.reset();
        if (octree_->computeRayKeys(sensor_origin, point, key_ray_)) {
            if (params_.max_free_space == 0.0) {
                free_cells->insert(key_ray_.begin(), key_ray_.end());
            } else {
                for (const auto& key : key_ray_) {
                    octomap::point3d voxel_coordinate = octree_->keyToCoord(key);
                    if ((voxel_coordinate - sensor_origin).norm() < params_.max_free_space ||
                        voxel_coordinate.z() > (sensor_origin.z() - params_.min_height_free_space)) {
                        free_cells->insert(key);
                    }
                }
            }
        }
        // Mark endpoing as occupied.
        octomap::OcTreeKey key;
        if (octree_->coordToKeyChecked(point, key)) {
            occupied_cells->insert(key);
        }
    } else {
        // If the ray is longer than the max range, just update free space.
        octomap::point3d new_end =
            sensor_origin +
            (point - sensor_origin).normalized() * params_.sensor_max_range;
        key_ray_.reset();
        if (octree_->computeRayKeys(sensor_origin, new_end, key_ray_)) {
            if (params_.max_free_space == 0.0) {
                free_cells->insert(key_ray_.begin(), key_ray_.end());
            } else {
                for (const auto& key : key_ray_) {
                    octomap::point3d voxel_coordinate = octree_->keyToCoord(key);
                    if ((voxel_coordinate - sensor_origin).norm() < params_.max_free_space ||
                        voxel_coordinate.z() > (sensor_origin.z() - params_.min_height_free_space)) {
                        free_cells->insert(key);
                    }
                }
            }
        }
    }
}

// void OctomapManager::castRaySecondary(
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
//     key_ray_secondary_.reset();
//     if (octree_->computeRayKeys(sensor_origin, ray_end, key_ray_secondary_)) {
//         if (params_.max_free_space == 0.0) {
//             viewed_cells->insert(key_ray_secondary_.begin(), key_ray_secondary_.end());
//         } else {
//             for (const auto& key : key_ray_secondary_) {
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

FrustumAngles OctomapManager::getCameraFOV(const CameraIntrinsics& intrinsics) const {
    double fov_h = 2 * std::atan(intrinsics.width / (2.0 * intrinsics.fx));   // Horizontal
    double fov_v = 2 * std::atan(intrinsics.height / (2.0 * intrinsics.fy));  // Vertical
    return {fov_h, fov_v};
}

void OctomapManager::markFrustumAsViewed(
    const Eigen::Isometry3d& T_G_sensor,
    const double fov_h,
    const double fov_v,
    const double max_range)
{
    const octomap::point3d sensor_origin = pointEigenToOctomap(T_G_sensor.translation());
    
    double half_fov_h = fov_h / 2.0;   // Horizontal
    double half_fov_v = fov_v / 2.0;  // Vertical
    
    // Sample rays across the frustum
    const int horizontal_rays = 40;
    const int vertical_rays = 30;
    
    for (int h = 0; h < horizontal_rays; ++h) {
        for (int v = 0; v < vertical_rays; ++v) {
            // Angular position within frustum
            double angle_h = -half_fov_h + (2.0 * half_fov_h * h) / (horizontal_rays - 1);
            double angle_v = -half_fov_v + (2.0 * half_fov_v * v) / (vertical_rays - 1);
            
            // Ray direction in sensor frame
            double horizontal_offset = -std::tan(angle_h); // -Y is right
            double vertical_offset = std::tan(angle_v);    // +Z is up
            Eigen::Vector3d ray(1.0, horizontal_offset, vertical_offset);
            ray.normalize();
            
            // Transform to world frame
            Eigen::Vector3d ray_world = T_G_sensor.rotation() * ray;
            Eigen::Vector3d endpoint_world = T_G_sensor.translation() + ray_world * max_range;
            octomap::point3d endpoint_octo = pointEigenToOctomap(endpoint_world);
            octomap::point3d ray_world_octo = pointEigenToOctomap(ray_world);
            
            // Cast ray and find where it hits occupied/unknown space
            octomap::point3d ray_end;
            bool hit_obstacle = octree_->castRay(sensor_origin, ray_world_octo, ray_end, false, max_range);
            
            // Mark all voxels along the ray as viewed (up to obstacle or max range)
            octomap::point3d actual_end = hit_obstacle ? ray_end : endpoint_octo;
            
            key_ray_.reset();
            if (octree_->computeRayKeys(sensor_origin, actual_end, key_ray_)) {
                viewed_voxels_.insert(key_ray_.begin(), key_ray_.end());
            }
        }
    }
}

void OctomapManager::updateOccupancy(
    octomap::KeySet* free_cells,
    octomap::KeySet* occupied_cells)
{
    if (free_cells == nullptr || occupied_cells == nullptr) {
        return;
    }

    // Mark occupied cells.
    for (const auto& key : *occupied_cells) {
        octree_->updateNode(key, true); // true = occupied

        // Remove any occupied cells from free cells - assume there are far fewer
        // occupied cells than free cells, so this is much faster than checking on
        // every free cell.
        auto it = free_cells->find(key);
        if (it != free_cells->end()) {
            free_cells->erase(it);
        }
    }

    // Mark free cells.
    for (const auto& key : *free_cells) {
        octree_->updateNode(key, false);  // false = free
    }
    octree_->updateInnerOccupancy();
}

void OctomapManager::reset() {
    double resolution = octree_->getResolution();
    octree_ = std::make_unique<octomap::OcTree>(resolution);
}

bool OctomapManager::saveMap(const std::string& filename) {
    return octree_->write(filename);
}

bool OctomapManager::loadMap(const std::string& filename) {
    auto loaded_tree = std::unique_ptr<octomap::OcTree>(
        dynamic_cast<octomap::OcTree*>(octomap::OcTree::read(filename)));
    
    if (loaded_tree) {
        octree_ = std::move(loaded_tree);
        return true;
    }
    return false;
}

void OctomapManager::getMapBounds(octomap::point3d& min, octomap::point3d& max) const {
    double x, y, z;
    octree_->getMetricMin(x, y, z);
    min = octomap::point3d(x, y, z);
    
    octree_->getMetricMax(x, y, z);
    max = octomap::point3d(x, y, z);
}

bool OctomapManager::isVoxelViewed(const octomap::OcTreeKey& key) const
{
    return viewed_voxels_.find(key) != viewed_voxels_.end();
}

bool OctomapManager::isPointViewed(const octomap::point3d& point) const
{
    octomap::OcTreeKey key;
    if (octree_->coordToKeyChecked(point, key)) {
        return isVoxelViewed(key);
    }
    return false;
}

} // namespace nbv_planner