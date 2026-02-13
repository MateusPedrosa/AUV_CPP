#include "nbv_planner/voxblox_manager.hpp"
#include <voxblox/integrator/tsdf_integrator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>

namespace nbv_planner {

voxblox::Point pointEigenToVoxblox(const Eigen::Vector3d& point)
{
    return voxblox::Point(point.x(), point.y(), point.z());
}

VoxbloxManager::VoxbloxManager(const VoxbloxParameters& params)
    : params_(params)
{
    // Configure TSDF layer
    voxblox::TsdfMap::Config config;
    config.tsdf_voxel_size = params_.voxel_size;
    config.tsdf_voxels_per_side = params_.voxels_per_side;
    
    tsdf_map_ = std::make_shared<voxblox::TsdfMap>(config);
    
    // Configure TSDF integrator
    voxblox::TsdfIntegratorBase::Config integrator_config;
    integrator_config.default_truncation_distance = params_.truncation_distance;
    integrator_config.max_ray_length_m = params_.max_ray_length;
    integrator_config.min_ray_length_m = params_.min_ray_length;
    integrator_config.voxel_carving_enabled = params_.voxel_carving_enabled;
    integrator_config.max_weight = params_.max_weight;
    integrator_config.use_const_weight = params_.use_const_weight;
    integrator_config.allow_clear = params_.allow_clear;
    integrator_config.start_voxel_subsampling_factor = params_.start_voxel_subsampling_factor;
    integrator_config.max_consecutive_ray_collisions = params_.max_consecutive_ray_collisions;
    integrator_config.clear_checks_every_n_frames = params_.clear_checks_every_n_frames;
    
    // Use merged integrator for better performance
    tsdf_integrator_ = std::make_shared<voxblox::MergedTsdfIntegrator>(
        integrator_config, 
        tsdf_map_->getTsdfLayerPtr());
}

void VoxbloxManager::insertPointCloudIntoMap(
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

     // Convert PCL to Voxblox pointcloud
    voxblox::Pointcloud voxblox_pointcloud;
    voxblox::Colors colors;  // Empty colors - not using color for now
    
    voxblox_pointcloud.reserve(transformed_cloud->size());
    for (const auto& point : transformed_cloud->points) {
        voxblox_pointcloud.push_back(voxblox::Point(point.x, point.y, point.z));
    }

    colors.resize(voxblox_pointcloud.size(), voxblox::Color(255, 255, 255));

    // Integrate the pointcloud
    // voxblox::Transformation uses double precision internally
    Eigen::Matrix3d rotation_matrix = T_G_sensor.rotation();
    Eigen::Vector3d translation_vector = T_G_sensor.translation();

    voxblox::Transformation T_G_C(
        voxblox::Rotation(Eigen::Quaterniond(rotation_matrix)),
        translation_vector);

    tsdf_integrator_->integratePointCloud(T_G_C, voxblox_pointcloud, colors);
}

VoxbloxManager::FrustumAngles VoxbloxManager::getCameraFOV(const VoxbloxManager::CameraIntrinsics& intrinsics) const {
    double fov_h = 2 * std::atan(intrinsics.width / (2.0 * intrinsics.fx));   // Horizontal
    double fov_v = 2 * std::atan(intrinsics.height / (2.0 * intrinsics.fy));  // Vertical
    return {fov_h, fov_v};
}

void VoxbloxManager::markFrustumAsViewed(
    const Eigen::Isometry3d& T_G_sensor,
    const double fov_h,
    const double fov_v,
    const double max_range)
{
    const voxblox::Point sensor_origin = pointEigenToVoxblox(T_G_sensor.translation());
    
    double half_fov_h = fov_h / 2.0;
    double half_fov_v = fov_v / 2.0;
    
    // Sample rays across the frustum
    const int horizontal_rays = 40;
    const int vertical_rays = 30;
    
    voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer = tsdf_map_->getTsdfLayerPtr();
    
    for (int h = 0; h < horizontal_rays; ++h) {
        for (int v = 0; v < vertical_rays; ++v) {
            // Angular position within frustum
            double angle_h = -half_fov_h + (2.0 * half_fov_h * h) / (horizontal_rays - 1);
            double angle_v = -half_fov_v + (2.0 * half_fov_v * v) / (vertical_rays - 1);
            
            // Ray direction in sensor frame
            double horizontal_offset = -std::tan(angle_h);
            double vertical_offset = std::tan(angle_v);
            Eigen::Vector3d ray(1.0, horizontal_offset, vertical_offset);
            ray.normalize();
            
            // Transform to world frame
            Eigen::Vector3d ray_world = T_G_sensor.rotation() * ray;
            
            // Cast ray and mark voxels as viewed
            voxblox::Point ray_direction(ray_world.x(), ray_world.y(), ray_world.z());
            
            // Manually traverse along the ray
            const float step_size = params_.voxel_size * 0.5;
            const int max_steps = static_cast<int>(max_range / step_size);
            
            for (int step = 0; step < max_steps; ++step) {
                float distance = step * step_size;
                voxblox::Point current_point = sensor_origin + ray_direction * distance;
                
                // Get the voxel at this point
                voxblox::BlockIndex block_index = 
                    tsdf_layer->computeBlockIndexFromCoordinates(current_point);
                
                voxblox::Block<voxblox::TsdfVoxel>::Ptr block = 
                    tsdf_layer->getBlockPtrByIndex(block_index);
                
                if (block) {
                    voxblox::VoxelIndex voxel_index = 
                        block->computeVoxelIndexFromCoordinates(current_point);
                    
                    voxblox::GlobalIndex global_index = 
                        voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
                            block_index, voxel_index, params_.voxels_per_side);
                    
                    viewed_voxels_.insert(global_index);
                    
                    // Check if we hit an occupied voxel (distance close to zero or negative)
                    voxblox::TsdfVoxel& voxel = block->getVoxelByVoxelIndex(voxel_index);
                    if (voxel.weight > 1e-6 && voxel.distance < params_.voxel_size) {
                        break;  // Stop at obstacle
                    }
                }
            }
        }
    }
}

bool VoxbloxManager::isOccupied(const voxblox::Point& point) const
{
    voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer = tsdf_map_->getTsdfLayerPtr();
    
    voxblox::TsdfVoxel* voxel = tsdf_layer->getVoxelPtrByCoordinates(point);
    
    if (voxel == nullptr || voxel->weight < 1e-6) {
        return false;  // Unknown space
    }
    
    // A voxel is considered occupied if distance is close to zero or negative
    return voxel->distance < params_.surface_distance_threshold;
}

bool VoxbloxManager::isKnown(const voxblox::Point& point) const
{
    voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer = tsdf_map_->getTsdfLayerPtr();
    
    voxblox::TsdfVoxel* voxel = tsdf_layer->getVoxelPtrByCoordinates(point);
    
    return voxel != nullptr && voxel->weight > 1e-6;
}

float VoxbloxManager::getDistance(const voxblox::Point& point) const
{
    voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer = tsdf_map_->getTsdfLayerPtr();
    
    voxblox::TsdfVoxel* voxel = tsdf_layer->getVoxelPtrByCoordinates(point);
    
    if (voxel == nullptr || voxel->weight < 1e-6) {
        return std::numeric_limits<float>::max();  // Unknown
    }
    
    return voxel->distance;
}

void VoxbloxManager::reset() {
    voxblox::TsdfMap::Config config;
    config.tsdf_voxel_size = params_.voxel_size;
    config.tsdf_voxels_per_side = params_.voxels_per_side;
    
    tsdf_map_ = std::make_shared<voxblox::TsdfMap>(config);
    
    voxblox::TsdfIntegratorBase::Config integrator_config;
    integrator_config.default_truncation_distance = params_.truncation_distance;
    integrator_config.max_ray_length_m = params_.max_ray_length;
    
    tsdf_integrator_ = std::make_shared<voxblox::MergedTsdfIntegrator>(
        integrator_config, 
        tsdf_map_->getTsdfLayerPtr());
    
    viewed_voxels_.clear();
}

bool VoxbloxManager::saveMap(const std::string& filename) {
    return voxblox::io::SaveLayer(tsdf_map_->getTsdfLayer(), filename);
}

bool VoxbloxManager::loadMap(const std::string& filename) {
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr loaded_layer;
    
    if (voxblox::io::LoadLayer<voxblox::TsdfVoxel>(filename, &loaded_layer)) {
        // Create new map with loaded layer
        tsdf_map_ = std::make_shared<voxblox::TsdfMap>(loaded_layer);
        
        // Recreate integrator
        voxblox::TsdfIntegratorBase::Config integrator_config;
        integrator_config.default_truncation_distance = params_.truncation_distance;
        integrator_config.max_ray_length_m = params_.max_ray_length;
        
        tsdf_integrator_ = std::make_shared<voxblox::MergedTsdfIntegrator>(
            integrator_config, 
            tsdf_map_->getTsdfLayerPtr());
        
        return true;
    }
    return false;
}

VoxbloxManager::MapBounds VoxbloxManager::getMapBounds() const {
    voxblox::Point min_point, max_point;
    
    // Get bounds from all allocated blocks
    voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer = tsdf_map_->getTsdfLayerPtr();
    
    bool first = true;
    voxblox::BlockIndexList blocks;
    tsdf_layer->getAllAllocatedBlocks(&blocks);
    
    for (const auto& block_index : blocks) {
        voxblox::Point block_origin = 
            voxblox::getOriginPointFromGridIndex(block_index, tsdf_layer->block_size());
        
        voxblox::Point block_max = block_origin + 
            voxblox::Point(tsdf_layer->block_size(), 
                          tsdf_layer->block_size(), 
                          tsdf_layer->block_size());
        
        if (first) {
            min_point = block_origin;
            max_point = block_max;
            first = false;
        } else {
            min_point = min_point.cwiseMin(block_origin);
            max_point = max_point.cwiseMax(block_max);
        }
    }
    
    if (first) {
        // No blocks allocated, return default bounds
        min_point = voxblox::Point(-10.0, -10.0, -10.0);
        max_point = voxblox::Point(10.0, 10.0, 10.0);
    }
    
    return {min_point, max_point};
}

bool VoxbloxManager::isVoxelViewed(const voxblox::GlobalIndex& index) const
{
    return viewed_voxels_.find(index) != viewed_voxels_.end();
}

bool VoxbloxManager::isPointViewed(const voxblox::Point& point) const
{
    voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer = tsdf_map_->getTsdfLayerPtr();
    
    voxblox::BlockIndex block_index = 
        tsdf_layer->computeBlockIndexFromCoordinates(point);
    
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block = 
        tsdf_layer->getBlockPtrByIndex(block_index);
    
    if (!block) {
        return false;
    }
    
    voxblox::VoxelIndex voxel_index = 
        block->computeVoxelIndexFromCoordinates(point);
    
    voxblox::GlobalIndex global_index = 
        voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
            block_index, voxel_index, params_.voxels_per_side);
    
    return isVoxelViewed(global_index);
}

voxblox::Layer<voxblox::TsdfVoxel>* VoxbloxManager::getTsdfLayer() const
{
    return tsdf_map_->getTsdfLayerPtr();
}

std::shared_ptr<voxblox::TsdfMap> VoxbloxManager::getTsdfMap() const
{
    return tsdf_map_;
}

} // namespace nbv_planner