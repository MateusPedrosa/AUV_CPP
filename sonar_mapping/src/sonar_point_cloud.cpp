#include "rclcpp/rclcpp.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <oculus_interfaces/msg/oculus_ping.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

using std::placeholders::_1;

class SonarPointCloud : public rclcpp::Node
{
public:
    SonarPointCloud() : Node("sonar_point_cloud") {
        // Parameters
        this->declare_parameter("sonar_topic", "/oceansim/robot/imaging_sonar");
        this->declare_parameter("pose_topic", "/oceansim/robot/pose");
        this->declare_parameter("resolution", 0.1); // 10cm voxels
        this->declare_parameter("min_range", 0.1); // Minimum sonar range in meters
        this->declare_parameter("max_range", 40.0); // Maximum sonar range in meters
        this->declare_parameter("min_intensity_short_range", 0.4); // Minimum intensity threshold for short range
        this->declare_parameter("min_intensity_long_range", 0.2); // Minimum intensity threshold for long range
        this->declare_parameter("horizontal_fov", 130.0); // Horizontal field of view in degrees
        this->declare_parameter("vertical_fov", 20.0); // Vertical field of view in degrees
        this->declare_parameter("filter_window_size", 7); // Window size s (must be even, s+1 total beams)
        this->declare_parameter("filter_distance_threshold", 2.0); // Max average distance threshold in meters
        this->declare_parameter("radius_search", 0.5); // 50cm radius
        this->declare_parameter("min_neighbors", 5);   // At least 5 neighbors

        std::string sonar_topic = this->get_parameter("sonar_topic").as_string();
        std::string pose_topic = this->get_parameter("pose_topic").as_string();
        resolution_ = this->get_parameter("resolution").as_double();
        min_range_ = this->get_parameter("min_range").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        min_intensity_short_range_ = this->get_parameter("min_intensity_short_range").as_double();
        min_intensity_long_range_ = this->get_parameter("min_intensity_long_range").as_double();
        h_fov_ = this->get_parameter("horizontal_fov").as_double() * M_PI / 180.0;
        v_fov_ = this->get_parameter("vertical_fov").as_double() * M_PI / 180.0;
        filter_window_size_ = this->get_parameter("filter_window_size").as_int();
        filter_distance_threshold_ = this->get_parameter("filter_distance_threshold").as_double();
        radius_search_ = this->get_parameter("radius_search").as_double();
        min_neighbors_ = this->get_parameter("min_neighbors").as_int();

        // Ensure window size is even
        if (filter_window_size_ % 2 != 0) {
            RCLCPP_WARN(this->get_logger(), "Filter window size must be even, adjusting %d to %d",
                        filter_window_size_, filter_window_size_ + 1);
            filter_window_size_++;
        }

        // TODO: check qos
        rclcpp::QoS qos_profile(1);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sonar_point_cloud", qos_profile);

        rclcpp::QoS qos(1);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        sonar_subscriber_ = this->create_subscription<oculus_interfaces::msg::OculusPing>(
            sonar_topic, qos,
            std::bind(&SonarPointCloud::pingCallback, this, _1)
        );
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Subscription<oculus_interfaces::msg::OculusPing>::SharedPtr sonar_subscriber_;
    double resolution_;
    double min_range_;
    double max_range_;
    double min_intensity_short_range_;
    double min_intensity_long_range_;
    double h_fov_;
    double v_fov_;
    int filter_window_size_;
    double filter_distance_threshold_;
    double radius_search_;
    int min_neighbors_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr applyRadiusFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
        if (input_cloud->empty()) return input_cloud;

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        
        outrem.setInputCloud(input_cloud);
        outrem.setRadiusSearch(radius_search_);
        outrem.setMinNeighborsInRadius(min_neighbors_);
        
        // Apply filter
        outrem.filter(*filtered_cloud);
        return filtered_cloud;
    }

    /**
     * @brief Filters sonar features for noise and outliers using averaged point distances.
     * * This method implements a moving window filter across consecutive sonar beams. 
     * It assumes that feature points belonging to physical objects should be spatially 
     * close across adjacent beams.
     * * The average distance value for a focus point $f_i^S$ is calculated using a window 
     * of size $s$ (totaling $s + 1$ consecutive beams):
     * * $$\bar{d}_i^f = \frac{1}{s+1} \sum_{j=i-\frac{s}{2}}^{i+\frac{s}{2}} \sqrt{(f_{i_x}^S - f_{j_x}^S)^2 + (f_{i_y}^S - f_{j_y}^S)^2}$$
     *
     * @param points_by_beam A vector of vectors containing points organized by their respective laser beam index.
     * @return pcl::PointCloud<pcl::PointXYZ> Filtered point cloud
     * * @note Source: "ROV-Based Autonomous Maneuvering for Ship Hull Inspection with Coverage Monitoring" (Cardaillac, A. et al.) - Equations 54 and 55
     */
    pcl::PointCloud<pcl::PointXYZ> filterPointCloud(const std::vector<std::vector<pcl::PointXYZ>>& points_by_beam) {
        pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

        int num_beams = points_by_beam.size();
        
        if (num_beams == 0) {
            return filtered_cloud;
        }

        int half_window = filter_window_size_ / 2;

        // Process each beam
        for (int i = 0; i < num_beams; i++) {
            const auto& beam_points = points_by_beam[i];
            
            // Process each point in the beam
            for (const auto& focus_point : beam_points) {
                // Calculate window boundaries
                int start_beam = std::max(0, i - half_window);
                int end_beam = std::min(num_beams - 1, i + half_window);
                
                // Compute average distance to surrounding points
                double sum_distances = 0.0;
                int count = 0;
                
                for (int j = start_beam; j <= end_beam; j++) {
                    for (const auto& neighbor_point : points_by_beam[j]) {
                        // Calculate Euclidean distance
                        double dx = focus_point.x - neighbor_point.x;
                        double dy = focus_point.y - neighbor_point.y;
                        double dz = focus_point.z - neighbor_point.z;
                        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                        
                        sum_distances += distance;
                        count++;
                    }
                }
                
                // Calculate average distance d_i^f = (1/(s+1)) * sum of distances
                double avg_distance = (count > 0) ? sum_distances / count : 0.0;
                
                // Keep point if average distance is below threshold
                // For present objects, feature points should be close to each other
                if (avg_distance <= filter_distance_threshold_) {
                    filtered_cloud.push_back(focus_point);
                }
            }
        }
        
        return filtered_cloud;
    }

    void pingCallback(const oculus_interfaces::msg::OculusPing::ConstSharedPtr& ping_msg) {
        const uint16_t n_beams  = ping_msg->n_beams;
        const uint16_t n_ranges = ping_msg->n_ranges;

        if (n_beams == 0 || n_ranges == 0) {
            RCLCPP_WARN(this->get_logger(), "Received ping with zero beams or ranges, skipping.");
            return;
        }

        // ---- Range resolution ----
        // ping_msg->range is the configured maximum range (metres).
        // range_res = range / n_ranges.
        // Fall back to max_range_ parameter if the field is zero.
        double total_range = ping_msg->range > 0.0 ? ping_msg->range : max_range_;
        double range_res   = total_range / static_cast<double>(n_ranges);

        // ---- Bearing array ----
        // Use them directly when available; fall back to uniform distribution over h_fov.
        std::vector<double> bearings(n_beams);
        if (static_cast<uint16_t>(ping_msg->bearings.size()) == n_beams) {
            for (uint16_t b = 0; b < n_beams; b++) {
                bearings[b] = static_cast<double>(ping_msg->bearings[b]) * M_PI / 180.0;
            }
        } else {
            RCLCPP_WARN_ONCE(this->get_logger(),
                             "Bearings array size (%zu) does not match n_beams (%u). "
                             "Falling back to uniform FOV distribution.",
                             ping_msg->bearings.size(), n_beams);
            for (uint16_t b = 0; b < n_beams; b++) {
                bearings[b] = (static_cast<double>(b) / (n_beams - 1) - 0.5) * h_fov_;
            }
        }

        // ---- Decode raw uint8 ping data ----
        // Data layout: n_ranges × n_beams, row-major.
        // ping_msg->step gives the number of bytes per sample.
        const size_t  bytes_per_sample = (ping_msg->step > 0) ? (ping_msg->step / n_beams) : 1;
        const size_t  expected_bytes   = static_cast<size_t>(n_ranges) * ping_msg->step;

        if (ping_msg->data.size() < expected_bytes) {
            RCLCPP_WARN(this->get_logger(),
                        "Ping data size mismatch: expected %zu bytes, got %zu. Skipping.",
                        expected_bytes, ping_msg->data.size());
            return;
        }

        RCLCPP_INFO_ONCE(this->get_logger(),
                         "First ping: n_beams=%u, n_ranges=%u, range=%.2f m, range_res=%.6f m, "
                         "step=%u, frequency=%.0f Hz, gain=%.1f",
                         n_beams, n_ranges, total_range, range_res,
                         bytes_per_sample, ping_msg->frequency, ping_msg->gain);

        const double max_intensity = (bytes_per_sample == 2) ? 65535.0 : 255.0;

        // ---- Build points_by_beam ----
        std::vector<std::vector<pcl::PointXYZ>> points_by_beam(n_beams);

        for (uint16_t r = 0; r < n_ranges; r++) {
            double range = r * range_res;
            if (range < min_range_ || range > max_range_) continue;

            for (uint16_t b = 0; b < n_beams; b++) {
                size_t idx = (static_cast<size_t>(r) * n_beams + b) * bytes_per_sample;

                double raw_val = 0.0;
                if (bytes_per_sample == 2) {
                    raw_val = static_cast<double>(
                        static_cast<uint16_t>(ping_msg->data[idx]) |
                        (static_cast<uint16_t>(ping_msg->data[idx + 1]) << 8));
                } else {
                    raw_val = static_cast<double>(ping_msg->data[idx]);
                }
                double intensity = raw_val / max_intensity;  // normalise to [0, 1]

                // Intensity thresholds (short range / long range)
                if ((range < 3.0 && intensity < min_intensity_short_range_) ||
                    (range >= 3.0 && intensity < min_intensity_long_range_)) {
                    continue;
                }

                // Frame: X=Forward, Y=Left, Z=Up
                double bearing = bearings[b];
                pcl::PointXYZ pt;
                pt.x = static_cast<float>(range * std::cos(bearing));
                pt.y = static_cast<float>(range * std::sin(bearing));
                pt.z = 0.0f;  // 2D sonar: assume constant depth plane
                points_by_beam[b].push_back(pt);
            }
        }

        // ---- Count before filtering ----
        size_t total_before = 0;
        for (const auto& beam : points_by_beam) total_before += beam.size();

        // ---- Apply filters ----
        pcl::PointCloud<pcl::PointXYZ> window_filtered = filterPointCloud(points_by_beam);
        pcl::PointCloud<pcl::PointXYZ>::Ptr window_filtered_ptr = window_filtered.makeShared();
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud = applyRadiusFilter(window_filtered_ptr);

        // ---- Logging ----
        size_t total_after   = final_cloud->size();
        size_t total_removed = total_before - total_after;
        double pct = total_before > 0 ? 100.0 * total_removed / total_before : 0.0;
        RCLCPP_DEBUG(this->get_logger(),
                     "Filtering: %zu → %zu points (removed %zu, %.1f%%)",
                     total_before, total_after, total_removed, pct);

        // ---- Publish ----
        if (!final_cloud->empty()) {
            sensor_msgs::msg::PointCloud2 out_msg;
            pcl::toROSMsg(*final_cloud, out_msg);
            out_msg.header.frame_id = "sonar_link";
            out_msg.header.stamp = ping_msg->header.stamp;
            point_cloud_pub_->publish(out_msg);
            RCLCPP_DEBUG(this->get_logger(), "Published point cloud with %zu points", total_after);
        } else {
            RCLCPP_WARN(this->get_logger(), "Point cloud is empty after filtering, nothing to publish.");
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SonarPointCloud>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}