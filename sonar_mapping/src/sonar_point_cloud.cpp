#include "rclcpp/rclcpp.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <oculus_interfaces/msg/ping.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
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
        this->declare_parameter("sonar_topic", "/sonar/ping");
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
        this->declare_parameter("morph_close_radius", 2); // Morphological closing kernel half-size; kernel = 2R+1 (0 to disable)
        this->declare_parameter("polar_image_size", 800);   // Output polar image width and height in pixels
        this->declare_parameter("image_publish_every", 10);  // Publish polar images every N pings (0 to disable)
        this->declare_parameter("min_consecutive_empty_beams", 50); // Min consecutive empty beams before adding sentinels

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
        morph_close_radius_ = this->get_parameter("morph_close_radius").as_int();
        polar_image_size_ = this->get_parameter("polar_image_size").as_int();
        image_publish_every_ = this->get_parameter("image_publish_every").as_int();
        min_consecutive_empty_beams_ = this->get_parameter("min_consecutive_empty_beams").as_int();

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
        point_cloud_pub_      = this->create_publisher<sensor_msgs::msg::PointCloud2>("sonar_point_cloud", qos_profile);
        sonar_image_raw_pub_  = this->create_publisher<sensor_msgs::msg::Image>("sonar_image/raw",      qos_profile);
        sonar_image_filt_pub_ = this->create_publisher<sensor_msgs::msg::Image>("sonar_image/filtered", qos_profile);

        rclcpp::QoS qos(1);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        sonar_subscriber_ = this->create_subscription<oculus_interfaces::msg::Ping>(
            sonar_topic, qos,
            std::bind(&SonarPointCloud::pingCallback, this, _1)
        );
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sonar_image_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sonar_image_filt_pub_;
    rclcpp::Subscription<oculus_interfaces::msg::Ping>::SharedPtr sonar_subscriber_;
    double resolution_;
    double min_range_;
    double max_range_;
    double min_intensity_short_range_;
    double min_intensity_long_range_;
    double h_fov_;
    double v_fov_;
    int filter_window_size_;
    double filter_distance_threshold_;
    int morph_close_radius_;
    int polar_image_size_;
    int image_publish_every_;
    int min_consecutive_empty_beams_;
    int ping_count_{0};

    /**
     * @brief Renders a sonar intensity grid (range × beam) into a polar fan image.
     *
     * Each output pixel is back-projected to (range, bearing) and looked up in the
     * intensity grid. The sonar origin is at the bottom-centre; forward is up.
     *
     * @param intensity_grid  CV_32FC1 matrix, rows=n_ranges, cols=n_beams, values in [0,1].
     * @param bearings        Per-beam bearing angles in radians.
     * @param range_res       Metres per range bin.
     * @param total_range     Maximum range in metres.
     * @return cv::Mat        CV_8UC3 BGR image of size polar_image_size_ × polar_image_size_.
     */
    cv::Mat renderPolarImage(const cv::Mat& intensity_grid,
                             const std::vector<double>& bearings,
                             double range_res,
                             double total_range)
    {
        const int S = polar_image_size_;

        // Precompute COLORMAP_HOT LUT (256 entries)
        cv::Mat lut_src(1, 256, CV_8UC1);
        for (int i = 0; i < 256; i++) lut_src.at<uint8_t>(0, i) = static_cast<uint8_t>(i);
        cv::Mat lut_bgr;
        cv::applyColorMap(lut_src, lut_bgr, cv::COLORMAP_HOT);

        cv::Mat polar(S, S, CV_8UC3, cv::Scalar(20, 20, 20));

        const double cx    = S * 0.5;
        const double cy    = S * 0.95;
        const double scale = cy / total_range;

        const int    n_ranges   = intensity_grid.rows;
        const int    n_beams    = intensity_grid.cols;
        const double bear_min   = bearings.front();
        const double bear_max   = bearings.back();
        const double bear_range = bear_max - bear_min;

        for (int v = 0; v < S; v++) {
            for (int u = 0; u < S; u++) {
                double dx =  (u - cx) / scale;
                double dy =  (cy - v) / scale;
                if (dy < 0.0) continue;

                double range   = std::sqrt(dx * dx + dy * dy);
                double bearing = std::atan2(dx, dy);

                if (range < min_range_ || range > total_range) continue;
                if (bearing < bear_min || bearing > bear_max)  continue;

                int r = static_cast<int>(range / range_res);
                if (r >= n_ranges) continue;

                double beam_frac = (bearing - bear_min) / bear_range * (n_beams - 1);
                int b = std::clamp(static_cast<int>(std::round(beam_frac)), 0, n_beams - 1);

                uint8_t grey = static_cast<uint8_t>(
                    std::clamp(intensity_grid.at<float>(r, b), 0.0f, 1.0f) * 255.0f);
                polar.at<cv::Vec3b>(v, u) = lut_bgr.at<cv::Vec3b>(0, grey);
            }
        }

        // Range rings overlay
        const cv::Scalar grid_col(70, 70, 70);
        const cv::Scalar text_col(180, 180, 180);
        const double ring_step = (total_range <= 10.0) ? 1.0
                               : (total_range <= 40.0) ? 5.0 : 10.0;

        double bear_min_deg = bear_min * 180.0 / M_PI;
        double bear_max_deg = bear_max * 180.0 / M_PI;
        double arc_start    = 270.0 - bear_max_deg;
        double arc_end      = 270.0 - bear_min_deg;

        for (double rng = ring_step; rng <= total_range; rng += ring_step) {
            int radius_px = static_cast<int>(rng * scale);
            cv::ellipse(polar,
                        cv::Point(static_cast<int>(cx), static_cast<int>(cy)),
                        cv::Size(radius_px, radius_px),
                        0.0, arc_start, arc_end, grid_col, 1, cv::LINE_AA);
            int label_y = static_cast<int>(cy) - radius_px - 3;
            if (label_y > 5 && label_y < S) {
                cv::putText(polar, std::to_string(static_cast<int>(rng)) + "m",
                            cv::Point(static_cast<int>(cx) + 4, label_y),
                            cv::FONT_HERSHEY_PLAIN, 0.8, text_col, 1, cv::LINE_AA);
            }
        }

        // Bearing lines every 10°
        for (int deg = -80; deg <= 80; deg += 10) {
            double angle_rad = deg * M_PI / 180.0;
            if (angle_rad < bear_min || angle_rad > bear_max) continue;
            int ex = static_cast<int>(cx + total_range * scale * std::sin(angle_rad));
            int ey = static_cast<int>(cy - total_range * scale * std::cos(angle_rad));
            cv::line(polar,
                     cv::Point(static_cast<int>(cx), static_cast<int>(cy)),
                     cv::Point(ex, ey), grid_col, 1, cv::LINE_AA);
            if (deg != 0) {
                cv::putText(polar, std::to_string(deg) + "\xc2\xb0",
                            cv::Point(ex, ey - 4),
                            cv::FONT_HERSHEY_PLAIN, 0.7, text_col, 1, cv::LINE_AA);
            }
        }

        return polar;
    }

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
     *                       The intensity field of each point must hold the beam index.
     * @return pcl::PointCloud<pcl::PointXYZI> Filtered point cloud (intensity = beam index).
     *
     * @note Adapted from: "ROV-Based Autonomous Maneuvering for Ship Hull Inspection with
     *       Coverage Monitoring" (Cardaillac, A. et al.) - Equations 54 and 55
     */
    pcl::PointCloud<pcl::PointXYZI> filterPointCloud(
        const std::vector<std::vector<pcl::PointXYZI>>& points_by_beam)
    {
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

    void pingCallback(const oculus_interfaces::msg::Ping::ConstSharedPtr& ping_msg) {
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
            // bearings are in 100ths of a degree → multiply by 0.01 to get degrees, then to radians
            for (uint16_t b = 0; b < n_beams; b++) {
                bearings[b] = static_cast<double>(ping_msg->bearings[b]) * 0.01 * M_PI / 180.0;
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

        // ---- Decode raw ping data into a normalised intensity grid ----
        // Data layout: n_ranges rows, each of size step bytes.
        // ping_msg->sample_size gives the number of bytes per sample.
        // When ping_msg->has_gains is true, each row starts with a 4-byte gain (uint32 LE)
        // before the n_beams * sample_size data bytes.
        const size_t bytes_per_sample = (ping_msg->sample_size > 0) ? ping_msg->sample_size : 1;
        const size_t gain_offset      = ping_msg->has_gains ? 4 : 0;
        const size_t expected_bytes   = static_cast<size_t>(n_ranges) * ping_msg->step;

        if (ping_msg->ping_data.size() < expected_bytes) {
            RCLCPP_WARN(this->get_logger(),
                        "Ping data size mismatch: expected %zu bytes, got %zu. Skipping.",
                        expected_bytes, ping_msg->ping_data.size());
            return;
        }

        RCLCPP_INFO_ONCE(this->get_logger(),
                         "First ping: n_beams=%u, n_ranges=%u, range=%.2f m, range_res=%.6f m, "
                         "sample_size=%zu, frequency=%.0f Hz, gain=%.1f",
                         n_beams, n_ranges, total_range, range_res,
                         bytes_per_sample, ping_msg->frequency, ping_msg->gain_percent);

        const double max_raw = (bytes_per_sample == 2) ? 65535.0 : 255.0;

        cv::Mat intensity_grid(n_ranges, n_beams, CV_32FC1);
        for (uint16_t r = 0; r < n_ranges; r++) {
            for (uint16_t b = 0; b < n_beams; b++) {
                size_t idx = static_cast<size_t>(r) * ping_msg->step + gain_offset + b * bytes_per_sample;
                double raw_val = 0.0;
                if (bytes_per_sample == 2) {
                    raw_val = static_cast<double>(
                        static_cast<uint16_t>(ping_msg->ping_data[idx]) |
                        (static_cast<uint16_t>(ping_msg->ping_data[idx + 1]) << 8));
                } else {
                    raw_val = static_cast<double>(ping_msg->ping_data[idx]);
                }
                intensity_grid.at<float>(r, b) = static_cast<float>(raw_val / max_raw);
            }
        }

        // ---- Morphological closing on the intensity image (optional) ----
        // Fills dark gaps between nearby bright pixels (echo continuity) without
        // darkening them, unlike a median filter. Kernel size = 2R+1; R=0 disables.
        cv::Mat intensity_grid_raw = intensity_grid.clone();

        if (morph_close_radius_ > 0) {
            int kernel_size = 2 * morph_close_radius_ + 1;
            cv::Mat kernel = cv::getStructuringElement(
                cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
            cv::Mat grid_8u;
            intensity_grid.convertTo(grid_8u, CV_8U, 255.0);
            cv::morphologyEx(grid_8u, grid_8u, cv::MORPH_CLOSE, kernel);
            grid_8u.convertTo(intensity_grid, CV_32F, 1.0 / 255.0);
        }

        // ---- Publish polar debug images (decimated) ----
        // Raw image: before morphological closing.
        // Filtered image: after morphological closing.
        ++ping_count_;
        if (image_publish_every_ > 0 && (ping_count_ % image_publish_every_) == 0) {
            auto stamp = ping_msg->header.stamp;

            auto publish_polar = [&](const cv::Mat& grid,
                                     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub)
            {
                cv::Mat polar = renderPolarImage(grid, bearings, range_res, total_range);
                std_msgs::msg::Header hdr;
                hdr.stamp    = stamp;
                hdr.frame_id = "sonar_link";
                auto img_msg = cv_bridge::CvImage(hdr, "bgr8", polar).toImageMsg();
                pub->publish(*img_msg);
            };

            publish_polar(intensity_grid_raw, sonar_image_raw_pub_);
            publish_polar(intensity_grid,     sonar_image_filt_pub_);
        }

        // ---- Build points_by_beam ----
        // Beam index b is temporarily stored in the intensity field for post-filter tracking.
        std::vector<std::vector<pcl::PointXYZI>> points_by_beam(n_beams);

        for (uint16_t r = 0; r < n_ranges; r++) {
            double range = r * range_res;
            if (range < min_range_ || range > max_range_) continue;

            for (uint16_t b = 0; b < n_beams; b++) {
                float intensity = intensity_grid.at<float>(r, b);

                // Filter by minimum intensity thresholds (short range / long range)
                if ((range < 3.0 && intensity < min_intensity_short_range_) ||
                    (range >= 3.0 && intensity < min_intensity_long_range_)) {
                    continue;
                }

                // Frame: X=Forward, Y=Left, Z=Up
                double bearing = bearings[b];
                pcl::PointXYZI pt;
                pt.x = static_cast<float>(range * std::cos(bearing));
                pt.y = static_cast<float>(range * std::sin(bearing));
                pt.z = 0.0f;  // 2D sonar: assume constant depth plane
                // Temporarily store beam index in intensity for post-filter sentinel tracking.
                pt.intensity = static_cast<float>(b);
                points_by_beam[b].push_back(pt);
            }
        }

        // ---- Pre-filter free-space sentinels ----
        // For beams that produced no returns at all, insert a sentinel point just beyond
        // total_range. OctoMap treats voxels along the ray up to (but not including) the
        // endpoint as free space, so a point at total_range + 1 marks the entire beam as free.
        // Only runs of at least min_consecutive_empty_beams_ are marked to avoid spurious
        // free-space rays from surface noise.
        pcl::PointCloud<pcl::PointXYZI> free_space_cloud;
        double sentinel_range = total_range + 1.0;
        {
            std::vector<bool> empty_flags(n_beams);
            for (int b = 0; b < n_beams; b++) empty_flags[b] = points_by_beam[b].empty();
            std::vector<bool> mask = sentinelMask(empty_flags, min_consecutive_empty_beams_);
            for (int b = 0; b < n_beams; b++) {
                if (!mask[b]) continue;
                double bearing = bearings[b];
                pcl::PointXYZI sentinel;
                sentinel.x = static_cast<float>(sentinel_range * std::cos(bearing));
                sentinel.y = static_cast<float>(sentinel_range * std::sin(bearing));
                sentinel.z = 0.0f;
                sentinel.intensity = 1.0f;
                free_space_cloud.push_back(sentinel);
            }
        }

        // ---- Count before filtering ----
        size_t total_before = 0;
        for (const auto& beam : points_by_beam) total_before += beam.size();

        // ---- Apply centroid-based window filter ----
        pcl::PointCloud<pcl::PointXYZI> filtered_cloud = filterPointCloud(points_by_beam);

        // ---- Rewrite intensity: 1 for nearest hit per beam, 0 for farther returns ----
        // The 2D sonar can have multiple range bins per beam passing the threshold.
        // Only the closest one is the actual surface hit; farther ones should not
        // mark voxels as occupied in OctoMap.
        //
        // Find the minimum squared distance to the sensor for each beam.
        std::vector<float> min_range_sq_for_beam(n_beams, std::numeric_limits<float>::max());
        for (const auto& p : filtered_cloud.points) {
            int b = static_cast<int>(p.intensity); // beam index stored earlier
            float dist_sq = p.x * p.x + p.y * p.y + p.z * p.z;
            if (dist_sq < min_range_sq_for_beam[b]) {
                min_range_sq_for_beam[b] = dist_sq;
            }
        }

        // Rewrite intensity — 1.0 for nearest point on the beam (surface hit),
        // 0.0 for farther returns on the same beam.
        float epsilon = 0.0001f;
        for (auto& p : filtered_cloud.points) {
            int b = static_cast<int>(p.intensity);
            float dist_sq = p.x * p.x + p.y * p.y + p.z * p.z;
            p.intensity = (std::abs(dist_sq - min_range_sq_for_beam[b]) < epsilon) ? 1.0f : 0.0f;
        }

        RCLCPP_DEBUG(this->get_logger(),
                     "Filtering: %zu → %zu points (removed %zu, %.1f%%)",
                     total_before, filtered_cloud.size(),
                     total_before - filtered_cloud.size(),
                     total_before > 0 ? 100.0 * (total_before - filtered_cloud.size()) / total_before : 0.0);

        // ---- Post-filter free-space sentinels ----
        // Add sentinels for beams that had returns before filtering but became empty
        // after the window filter removed all their points.
        {
            std::vector<bool> empty_post(n_beams);
            for (int b = 0; b < n_beams; b++) {
                empty_post[b] = (!points_by_beam[b].empty() &&
                                 min_range_sq_for_beam[b] == std::numeric_limits<float>::max());
            }
            std::vector<bool> mask = sentinelMask(empty_post, min_consecutive_empty_beams_);
            for (int b = 0; b < n_beams; b++) {
                if (!mask[b]) continue;
                double bearing = bearings[b];
                pcl::PointXYZI sentinel;
                sentinel.x = static_cast<float>(sentinel_range * std::cos(bearing));
                sentinel.y = static_cast<float>(sentinel_range * std::sin(bearing));
                sentinel.z = 0.0f;
                sentinel.intensity = 1.0f;
                filtered_cloud.push_back(sentinel);
            }
        }

        // Merge pre-filter free-space sentinels. Sentinels are not passed through
        // the window filter — they are synthetic rays, not real returns.
        filtered_cloud += free_space_cloud;

        RCLCPP_DEBUG(this->get_logger(),
                     "Free-space sentinels: %zu rays marked at range %.1fm",
                     free_space_cloud.size(), sentinel_range);

        // ---- Publish ----
        if (!filtered_cloud.empty()) {
            sensor_msgs::msg::PointCloud2 out_msg;
            pcl::toROSMsg(filtered_cloud, out_msg);
            out_msg.header.frame_id = "sonar_link";
            out_msg.header.stamp = ping_msg->header.stamp;
            point_cloud_pub_->publish(out_msg);
            RCLCPP_DEBUG(this->get_logger(), "Published point cloud with %zu points", filtered_cloud.size());
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
