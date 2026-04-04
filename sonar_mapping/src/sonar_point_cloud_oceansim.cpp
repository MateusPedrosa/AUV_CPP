#include "rclcpp/rclcpp.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
// #include <pcl/filters/radius_outlier_removal.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

using std::placeholders::_1;

// TODO: use OculusPing message type

// TODO: When vertical_arc_points > 1, add missing points after filtering

class SonarPointCloud : public rclcpp::Node
{
public:
    SonarPointCloud() : Node("sonar_point_cloud_oceansim") {
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
        this->declare_parameter("vertical_arc_points", 1); // Number of points to distribute along vertical arc (n)
        this->declare_parameter("min_consecutive_empty_beams", 5); // Min consecutive empty beams before adding sentinels
        this->declare_parameter("median_filter_radius", 1); // R in paper; kernel size = 2R+1 (0 to disable)

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
        vertical_arc_points_ = this->get_parameter("vertical_arc_points").as_int();
        min_consecutive_empty_beams_ = this->get_parameter("min_consecutive_empty_beams").as_int();
        median_filter_radius_ = this->get_parameter("median_filter_radius").as_int();

        // Ensure window size is even
        if (filter_window_size_ % 2 != 0) {
            RCLCPP_WARN(this->get_logger(), "Filter window size must be even, adjusting %d to %d",
                        filter_window_size_, filter_window_size_ + 1);
            filter_window_size_++;
        }

        // TODO: check qos
        rclcpp::QoS qos_profile(1);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sonar_point_cloud", qos_profile);

        rclcpp::QoS qos(1);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        sonar_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            sonar_topic, qos,
            std::bind(&SonarPointCloud::convertImageToPointCloud, this, _1)
        );
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sonar_subscriber_;
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
    int vertical_arc_points_;
    int min_consecutive_empty_beams_;
    int median_filter_radius_;

    // pcl::PointCloud<pcl::PointXYZI>::Ptr applyRadiusFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud) {
    //     if (input_cloud->empty()) return input_cloud;

    //     pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    //     pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
        
    //     outrem.setInputCloud(input_cloud);
    //     outrem.setRadiusSearch(radius_search_);
    //     outrem.setMinNeighborsInRadius(min_neighbors_);
        
    //     // Apply filter
    //     outrem.filter(*filtered_cloud);
    //     return filtered_cloud;
    // }

    // Returns a mask indicating which beams belong to a run of at least min_run consecutive
    // empty beams. Single (or few) empty beams caused by surface noise are excluded.
    std::vector<bool> sentinelMask(const std::vector<bool>& empty_flags, int min_run) {
        int n = static_cast<int>(empty_flags.size());
        std::vector<bool> mask(n, false);
        int i = 0;
        while (i < n) {
            if (!empty_flags[i]) { i++; continue; }
            int j = i;
            while (j < n && empty_flags[j]) j++;
            if (j - i >= min_run) {
                for (int k = i; k < j; k++) mask[k] = true;
            }
            i = j;
        }
        return mask;
    }

    /**
     * @brief Filters sonar features for noise and outliers using averaged point distances.
     *
     * This method implements a moving window filter across consecutive sonar beams.
     * It assumes that feature points belonging to physical objects should be spatially
     * close across adjacent beams.
     *
     * The average distance value for a focus point $f_i^S$ is approximated using the
     * centroid of each neighbouring beam's point set, rather than all individual points.
     * For each beam j in the window, the centroid $\bar{p}_j$ is:
     *
     *   $\bar{p}_j = \frac{1}{|B_j|} \sum_{p \in B_j} p$
     *
     * The filtered distance metric is then:
     *
     *   $\bar{d}_i^f = \frac{1}{s+1} \sum_{j=i-s/2}^{i+s/2} \| f_i^S - \bar{p}_j \|$
     *
     * This reduces the inner loop from O(points_per_beam × window_size) to O(window_size)
     * per focus point, while preserving the spatial coherence check from the original.
     * Beams with no returns are skipped (they contribute no centroid to the average).
     *
     * @param points_by_beam A vector of vectors containing points organized by beam index.
     * @return pcl::PointCloud<pcl::PointXYZI> Filtered point cloud
     *
     * @note Adapted from: "ROV-Based Autonomous Maneuvering for Ship Hull Inspection with
     *       Coverage Monitoring" (Cardaillac, A. et al.) - Equations 54 and 55
     */
    pcl::PointCloud<pcl::PointXYZI> filterPointCloud(
        const std::vector<std::vector<pcl::PointXYZI>>& points_by_beam) {
        pcl::PointCloud<pcl::PointXYZI> filtered_cloud;

        int num_beams = points_by_beam.size();

        if (num_beams == 0) {
            return filtered_cloud;
        }

        // Pre-compute one centroid per beam. Beams with no returns get a sentinel
        // flag (has_centroid = false) and are excluded from the distance average.
        struct BeamCentroid {
            double x{0.0}, y{0.0}, z{0.0};
            bool has_centroid{false};
        };

        std::vector<BeamCentroid> centroids(num_beams);
        for (int i = 0; i < num_beams; i++) {
            const auto& pts = points_by_beam[i];
            if (pts.empty()) continue;

            double sx = 0.0, sy = 0.0, sz = 0.0;
            for (const auto& p : pts) {
                sx += p.x;
                sy += p.y;
                sz += p.z;
            }
            double inv_n = 1.0 / static_cast<double>(pts.size());
            centroids[i] = {sx * inv_n, sy * inv_n, sz * inv_n, true};
        }

        int half_window = filter_window_size_ / 2;

        // For each focus point, compute the average distance to the centroids of
        // the surrounding beams (window of size filter_window_size_ + 1).
        for (int i = 0; i < num_beams; i++) {
            const auto& beam_points = points_by_beam[i];
            if (beam_points.empty()) continue;

            int start_beam = std::max(0, i - half_window);
            int end_beam   = std::min(num_beams - 1, i + half_window);

            for (const auto& focus_point : beam_points) {
                double sum_distances = 0.0;
                int count = 0;

                for (int j = start_beam; j <= end_beam; j++) {
                    if (!centroids[j].has_centroid) continue;

                    double dx = focus_point.x - centroids[j].x;
                    double dy = focus_point.y - centroids[j].y;
                    double dz = focus_point.z - centroids[j].z;
                    sum_distances += std::sqrt(dx*dx + dy*dy + dz*dz);
                    count++;
                }

                // Keep point if average distance to neighbour centroids is below threshold
                double avg_distance = (count > 0) ? sum_distances / count : 0.0;
                if (avg_distance <= filter_distance_threshold_) {
                    filtered_cloud.push_back(focus_point);
                }
            }
        }

        return filtered_cloud;
    }

    void convertImageToPointCloud(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg) {
        // Convert ROS Image to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat sonar_image = cv_ptr->image;

        // ---- Median filter on the intensity image ----
        // Kernel size = 2R+1 per the paper; R=0 disables the filter.
        // cv::medianBlur requires CV_8U input, so we convert to 8-bit, blur, then back.
        if (median_filter_radius_ > 0) {
            int kernel_size = 2 * median_filter_radius_ + 1;
            cv::Mat grid_8u;
            sonar_image.convertTo(grid_8u, CV_8U, 255.0);
            cv::medianBlur(grid_8u, grid_8u, kernel_size);
            grid_8u.convertTo(sonar_image, CV_32F, 1.0 / 255.0);
        }

        int rows = sonar_image.rows;
        int cols = sonar_image.cols;

        // Calculate range resolution from image dimensions
        // The sensor creates bins using np.arange(min_range, max_range, range_res)
        // which produces approximately (max_range - min_range) / range_res bins
        // Therefore: range_res = (max_range - min_range) / rows
        double range_res = (max_range_ - min_range_) / static_cast<double>(rows);

        RCLCPP_INFO_ONCE(this->get_logger(), 
                        "Sonar image: %dx%d, range: %.2f-%.2fm, derived range_res: %.6fm",
                        cols, rows, min_range_, max_range_, range_res);

        std::vector<std::vector<pcl::PointXYZI>> points_by_beam(cols);

        // M750d produces 2D range-bearing data at constant depth
        for (int c = 0; c < cols; c++) {
            for (int r = 0; r < rows; r++) {
                float intensity = sonar_image.at<float>(r, c);

                // Calculate range (distance from sonar)
                // Each row r corresponds to: min_range + (r * range_res)
                // where range_res is derived from image dimensions
                double range = min_range_ + (r * range_res);

                // Filter by minimum intensity thresholds
                if ((range < 3.0 && intensity < min_intensity_short_range_) ||
                    (range >= 3.0 && intensity < min_intensity_long_range_)) {
                    continue;
                }

                // if (range > 5.0) {
                //     continue; // Skip points outside valid range
                // }

                // Calculate bearing angle (horizontal)
                // Assuming center column is 0 degrees, spreading across h_fov
                double bearing = (static_cast<double>(c) / cols - 0.5) * h_fov_;

                // The sonar gives range and bearing but the elevation (pitch) is ambiguous —
                // the return could originate from anywhere along a vertical arc of [-v_fov_/2, +v_fov/2].
                // We model this uncertainty by distributing n points evenly along that arc.
                //
                // For a given elevation angle phi, the 3D position is:
                //   x = range * cos(phi) * cos(bearing)
                //   y = range * cos(phi) * sin(bearing)
                //   z = range * sin(phi)
                //
                // At phi = 0 (horizontal) this reduces to the original x/y projection.
                int n = vertical_arc_points_;
                for (int k = 0; k < n; k++) {
                    // Elevation angle: uniformly sampled across [-v_fov_/2, +v_fov_/2]
                    double elevation = (n > 1)
                        ? (-v_fov_ / 2.0 + k * v_fov_ / (n - 1))
                        : 0.0;

                    double cos_elev = cos(elevation);

                    pcl::PointXYZI point;
                    point.x = range * cos_elev * cos(bearing);
                    point.y = range * cos_elev * sin(bearing);
                    point.z = range * sin(elevation);
                    // Temporarily store the beam (column) index in the intensity field so we can
                    // recover it after filtering. When vertical_arc_points > 1, multiple points
                    // share the same beam index — we later use it to identify the closest one.
                    point.intensity = static_cast<float>(c);

                    points_by_beam[c].push_back(point);
                }
            }
        }

        // For beams with no returns, insert a sentinel point just beyond max_range.
        // OctoMap treats the voxels along a ray up to (but not including) the endpoint
        // as free space, so a point at max_range + 1 marks the entire beam as free.
        // Only beams belonging to a run of at least min_consecutive_empty_beams_ consecutive
        // empty beams get a sentinel — isolated gaps from surface noise are ignored.
        pcl::PointCloud<pcl::PointXYZI> free_space_cloud;
        double sentinel_range = max_range_ + 1.0;
        {
            std::vector<bool> empty_flags(cols);
            for (int c = 0; c < cols; c++) empty_flags[c] = points_by_beam[c].empty();
            std::vector<bool> sentinel_mask = sentinelMask(empty_flags, min_consecutive_empty_beams_);
            for (int c = 0; c < cols; c++) {
                if (!sentinel_mask[c]) continue;

                double bearing = (static_cast<double>(c) / cols - 0.5) * h_fov_;
                int n = vertical_arc_points_;
                for (int k = 0; k < n; k++) {
                    double elevation = (n > 1)
                        ? (-v_fov_ / 2.0 + k * v_fov_ / (n - 1))
                        : 0.0;

                    double cos_elev = cos(elevation);
                    pcl::PointXYZI sentinel;
                    sentinel.x = sentinel_range * cos_elev * cos(bearing);
                    sentinel.y = sentinel_range * cos_elev * sin(bearing);
                    sentinel.z = sentinel_range * sin(elevation);
                    sentinel.intensity = 1.0f;
                    free_space_cloud.push_back(sentinel);
                }
            }
        }

        // Count total points before filtering
        size_t total_points_before = 0;
        for (const auto& beam : points_by_beam) {
            total_points_before += beam.size();
        }

        pcl::PointCloud<pcl::PointXYZI> filtered_cloud = filterPointCloud(points_by_beam);

        // pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud = applyRadiusFilter(window_filtered_cloud_ptr);

        filtered_cloud.header.frame_id = "sonar_link";

        // When vertical_arc_points > 1, each sonar return spawns multiple points distributed
        // along the vertical arc. All copies share the same beam index (stored in intensity)
        // and the same range, so they are all at the same distance from the sensor. Only the
        // closest point per beam should be treated as a surface hit (intensity = 1); the others
        // are elevation-arc duplicates and should be marked as non-hits (intensity = 0) so that
        // OctoMap does not log the same obstacle multiple times at different heights.
        //
        // Pass 1: find the minimum squared distance to the sensor for each beam.
        std::vector<float> min_range_sq_for_col(cols, std::numeric_limits<float>::max());
        for (size_t i = 0; i < filtered_cloud.size(); i++) {
            const auto& p = filtered_cloud.points[i];
            int c = static_cast<int>(p.intensity); // beam index stored earlier
            float dist_sq = p.x * p.x + p.y * p.y + p.z * p.z;
            if (dist_sq < min_range_sq_for_col[c]) {
                min_range_sq_for_col[c] = dist_sq;
            }
        }

        // Pass 2: rewrite intensity to its final meaning — 1 for the nearest point on a beam
        // (the actual surface hit), 0 for farther arc duplicates on the same beam.
        // epsilon guards the floating-point equality check; both passes use the same formula
        // on the same float values so the results are bitwise identical in practice.
        float epsilon = 0.0001f;
        for (size_t i = 0; i < filtered_cloud.size(); i++) {
            auto& p = filtered_cloud.points[i];
            int c = static_cast<int>(p.intensity);
            float dist_sq = p.x * p.x + p.y * p.y + p.z * p.z;
            p.intensity = (std::abs(dist_sq - min_range_sq_for_col[c]) < epsilon) ? 1.0f : 0.0f;
        }

        RCLCPP_DEBUG(this->get_logger(), "Filtered cloud from %zu beams to %zu points", points_by_beam.size(), filtered_cloud.size());

        size_t points_after = filtered_cloud.size();
        size_t points_removed = total_points_before - points_after;
        double filter_percentage = total_points_before > 0 ? 
            (100.0 * points_removed / total_points_before) : 0.0;

        RCLCPP_DEBUG(this->get_logger(), 
                    "Filtering: %zu points before -> %zu points after (removed %zu points, %.1f%%)",
                    total_points_before, points_after, points_removed, filter_percentage);

        // Add sentinel points in beams that became empty after filtering.
        // Only add sentinels for runs of at least min_consecutive_empty_beams_ consecutive
        // empty beams — isolated gaps from surface noise are ignored.
        {
            std::vector<bool> empty_post(cols);
            for (int c = 0; c < cols; c++) {
                empty_post[c] = (min_range_sq_for_col[c] == std::numeric_limits<float>::max());
            }
            std::vector<bool> sentinel_mask = sentinelMask(empty_post, min_consecutive_empty_beams_);
            for (int c = 0; c < cols; c++) {
                if (!sentinel_mask[c]) continue;

                double bearing = (static_cast<double>(c) / cols - 0.5) * h_fov_;
                int n = vertical_arc_points_;

                for (int k = 0; k < n; k++) {
                    double elevation = (n > 1) ? (-v_fov_ / 2.0 + k * v_fov_ / (n - 1)) : 0.0;

                    pcl::PointXYZI sentinel;
                    sentinel.x = sentinel_range * cos(elevation) * cos(bearing);
                    sentinel.y = sentinel_range * cos(elevation) * sin(bearing);
                    sentinel.z = sentinel_range * sin(elevation);
                    sentinel.intensity = 1.0f;
                    filtered_cloud.push_back(sentinel);
                }
            }
        }

        // Merge free-space sentinels with the filtered occupancy cloud.
        // Sentinels are intentionally not passed through the outlier filter —
        // they are synthetic rays, not real returns, and should always be kept.
        filtered_cloud += free_space_cloud;

        RCLCPP_DEBUG(this->get_logger(),
                    "Free-space sentinels: %zu empty beams marked at range %.1fm",
                    free_space_cloud.size(), sentinel_range);

        // Publish filtered point cloud
        if (!filtered_cloud.empty()) {
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(filtered_cloud, output_msg);
            output_msg.header.frame_id = "sonar_link";
            output_msg.header.stamp = this->now();

            point_cloud_pub_->publish(output_msg);
            RCLCPP_DEBUG(this->get_logger(), "Published point cloud with %zu points", filtered_cloud.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "Point cloud is empty, nothing to publish.");
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