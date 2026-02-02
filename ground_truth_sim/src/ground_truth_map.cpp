#include "rclcpp/rclcpp.hpp"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

class VoxelMapGroundTruth : public rclcpp::Node
{
public:
    VoxelMapGroundTruth() : Node("voxel_map_ground_truth") {
        // Parameters
        this->declare_parameter("ply_path", "");
        this->declare_parameter("resolution", 0.1); // 10cm voxels

        std::string ply_path = this->get_parameter("ply_path").as_string();
        double resolution = this->get_parameter("resolution").as_double();

        // Initialize OctoMap
        ground_truth_map_ = std::make_shared<octomap::OcTree>(resolution);

        // Setup Publisher (Latched QoS)
        // We use "transient_local" durability so late joiners (like RViz) still get the map
        rclcpp::QoS qos_profile(1);
        qos_profile.transient_local(); 
        gt_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_truth_point_cloud", qos_profile);

        // Load the Map
        if (ply_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No PLY path provided! Set the 'ply_path' parameter.");
        } else {
            loadPlyMap(ply_path);
            publishMap();

            // Publish periodically for late subscribers
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&VoxelMapGroundTruth::publishMap, this)
            );
        }
    }

private:
    std::shared_ptr<octomap::OcTree> ground_truth_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr gt_pub_;
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    rclcpp::TimerBase::SharedPtr timer_;

    void loadPlyMap(const std::string& path) {
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(path, cloud_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file: %s", path.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loading %lu points from PLY...", cloud_.points.size());

        for (const auto& p : cloud_.points) {
            // 'true' here enables lazy evaluation (faster loading)
            // ground_truth_map_->updateNode(octomap::point3d(p.x, p.y, p.z), true);

            // Directly set as occupied (bypasses probabilistic updates)
            octomap::OcTreeNode* node = ground_truth_map_->updateNode(octomap::point3d(p.x, p.y, p.z), true);
            if (node) {
                // Set to maximum occupancy (log-odds)
                node->setLogOdds(ground_truth_map_->getClampingThresMaxLog());
            }
        }
        
        // Must call this after batch update!
        // ground_truth_map_->updateInnerOccupancy();
        RCLCPP_INFO(this->get_logger(), "OctoMap created with %zu nodes.", ground_truth_map_->size());
    }

    void publishMap() {
        if (cloud_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Map is empty, nothing to publish.");
            return;
        }

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(cloud_, msg);
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        
        gt_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published Ground Truth Map (%lu points)", cloud_.size());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VoxelMapGroundTruth>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}