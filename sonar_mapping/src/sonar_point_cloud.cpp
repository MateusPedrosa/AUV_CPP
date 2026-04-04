#include "rclcpp/rclcpp.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <oculus_interfaces/msg/oculus_ping.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cv_bridge/cv_bridge.hpp>
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
        this->declare_parameter("median_filter_radius", 1); // R in paper; kernel size = 2R+1 (0 to disable)
        this->declare_parameter("polar_image_size", 800);   // Output polar image width and height in pixels
        this->declare_parameter("adaptive_threshold_offset", -0.1); // O in paper eq. (7); in [0,1] normalised intensity
        this->declare_parameter("image_publish_every", 10);  // Publish polar images every N pings (0 to disable)

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
        median_filter_radius_ = this->get_parameter("median_filter_radius").as_int();
        polar_image_size_ = this->get_parameter("polar_image_size").as_int();
        adaptive_threshold_offset_ = this->get_parameter("adaptive_threshold_offset").as_double();
        image_publish_every_ = this->get_parameter("image_publish_every").as_int();

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
        sonar_image_raw_pub_  = this->create_publisher<sensor_msgs::msg::Image>("sonar_image/raw",      qos_profile);
        sonar_image_filt_pub_ = this->create_publisher<sensor_msgs::msg::Image>("sonar_image/filtered", qos_profile);

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
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sonar_image_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sonar_image_filt_pub_;
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
    int median_filter_radius_;
    int polar_image_size_;
    double adaptive_threshold_offset_;
    int image_publish_every_;
    int ping_count_{0};

    /**
     * @brief Renders a sonar intensity grid (range × beam) into a human-readable polar image.
     *
     * Each output pixel is back-projected to (range, bearing) and looked up in the
     * intensity grid. The sonar origin is at the bottom-centre; forward is up.
     * A precomputed COLORMAP_HOT LUT is used so the per-pixel cost is a single table
     * lookup rather than an OpenCV call.
     *
     * @param intensity_grid  CV_32FC1 matrix, rows=n_ranges, cols=n_beams, values in [0,1].
     * @param bearings        Per-beam bearing angles in radians.
     * @param range_res       Metres per range bin.
     * @param total_range     Maximum range in metres.
     * @return cv::Mat        CV_8UC3 BGR image of size polar_image_size_ × polar_image_size_.
     */
    /**
     * @brief Binarizes the intensity grid using a per-pixel adaptive threshold.
     *
     * Implements equations (7) and (8) from the paper:
     *   T[x_{i,j}] = (min(N[x_{i,j}]) + max(N[x_{i,j}])) / 2  -  O
     *   y_{i,j}    = 1  if x_{i,j} >= T[x_{i,j}],  else 0
     *
     * The neighbourhood N[x_{i,j}] is a (2R+1)×(2R+1) window centred on (i,j),
     * with R = median_filter_radius_ (same window used by the median filter).
     * Border pixels use the clamped neighbourhood (BORDER_REPLICATE).
     *
     * @param grid  CV_32FC1 input, values in [0,1].
     * @return      CV_32FC1 binary image: 1.0 where pixel passes, 0.0 elsewhere.
     */
    cv::Mat applyAdaptiveThreshold(const cv::Mat& grid)
    {
        const int R = (median_filter_radius_ > 0) ? median_filter_radius_ : 1;
        const int rows = grid.rows;
        const int cols = grid.cols;

        cv::Mat result(rows, cols, CV_32FC1, cv::Scalar(0.0f));

        // Precompute min and max over each (2R+1)×(2R+1) neighbourhood using
        // OpenCV's efficient morphological erosion (min) and dilation (max).
        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT, cv::Size(2 * R + 1, 2 * R + 1));

        cv::Mat local_min, local_max;
        cv::erode (grid, local_min, kernel, cv::Point(-1,-1), 1, cv::BORDER_REPLICATE);
        cv::dilate(grid, local_max, kernel, cv::Point(-1,-1), 1, cv::BORDER_REPLICATE);

        // T[i,j] = (min + max) / 2 - O   (eq. 7)
        // y[i,j] = 1 if x[i,j] >= T[i,j] (eq. 8)
        const float O = static_cast<float>(adaptive_threshold_offset_);
        for (int r = 0; r < rows; r++) {
            const float* row_grid = grid.ptr<float>(r);
            const float* row_min  = local_min.ptr<float>(r);
            const float* row_max  = local_max.ptr<float>(r);
            float*       row_out  = result.ptr<float>(r);

            for (int b = 0; b < cols; b++) {
                float threshold = (row_min[b] + row_max[b]) * 0.5f - O;
                row_out[b] = (row_grid[b] >= threshold) ? 1.0f : 0.0f;
            }
        }

        return result;
    }

    cv::Mat renderPolarImage(const cv::Mat& intensity_grid,
                             const std::vector<double>& bearings,
                             double range_res,
                             double total_range)
    {
        const int S = polar_image_size_;

        // ---- Precompute COLORMAP_HOT LUT (256 entries) ----
        cv::Mat lut_src(1, 256, CV_8UC1);
        for (int i = 0; i < 256; i++) lut_src.at<uint8_t>(0, i) = static_cast<uint8_t>(i);
        cv::Mat lut_bgr;
        cv::applyColorMap(lut_src, lut_bgr, cv::COLORMAP_HOT);  // 1×256 CV_8UC3

        cv::Mat polar(S, S, CV_8UC3, cv::Scalar(20, 20, 20));

        // Sonar origin: bottom-centre. Leave a small bottom margin so the
        // origin is not right on the image edge (avoids dy=0 instability).
        const double cx    = S * 0.5;
        const double cy    = S * 0.95;              // 95% down
        const double scale = cy / total_range;      // pixels per metre

        const int    n_ranges   = intensity_grid.rows;
        const int    n_beams    = intensity_grid.cols;
        const double bear_min   = bearings.front();
        const double bear_max   = bearings.back();
        const double bear_range = bear_max - bear_min;

        for (int v = 0; v < S; v++) {
            for (int u = 0; u < S; u++) {
                // Back-project pixel → (range, bearing)
                // dx: rightward offset in metres; dy: forward (upward) in metres
                double dx =  (u - cx) / scale;
                double dy =  (cy - v) / scale;   // cy-v so upward = positive

                // Only consider pixels in the forward hemisphere (dy >= 0)
                if (dy < 0.0) continue;

                double range   = std::sqrt(dx * dx + dy * dy);
                double bearing = std::atan2(dx, dy);  // 0 = forward, +ve = right

                if (range < min_range_ || range > total_range) continue;
                if (bearing < bear_min || bearing > bear_max)  continue;

                int r = static_cast<int>(range / range_res);
                if (r >= n_ranges) continue;

                double beam_frac = (bearing - bear_min) / bear_range * (n_beams - 1);
                int b = static_cast<int>(std::round(beam_frac));
                b = std::clamp(b, 0, n_beams - 1);

                // LUT lookup — single table read instead of per-pixel applyColorMap
                uint8_t grey = static_cast<uint8_t>(
                    std::clamp(intensity_grid.at<float>(r, b), 0.0f, 1.0f) * 255.0f);
                polar.at<cv::Vec3b>(v, u) = lut_bgr.at<cv::Vec3b>(0, grey);
            }
        }

        // ---- Grid overlay: range rings ----
        const cv::Scalar grid_col(70, 70, 70);
        const cv::Scalar text_col(180, 180, 180);
        const double ring_step = (total_range <= 10.0) ? 1.0
                               : (total_range <= 40.0) ? 5.0 : 10.0;

        // OpenCV ellipse angles: 0° = 3 o'clock, CW positive.
        // Our fan opens upward from cx,cy. The left edge of the fan is at
        // (90° + bearing_min_deg) CW from 3-o'clock = 90 - bearing_min_deg CCW.
        // Easier: start_angle_cv = 270 - bear_max_deg, end_angle_cv = 270 - bear_min_deg
        double bear_min_deg = bear_min * 180.0 / M_PI;
        double bear_max_deg = bear_max * 180.0 / M_PI;
        double arc_start    = 270.0 - bear_max_deg;
        double arc_end      = 270.0 - bear_min_deg;

        for (double rng = ring_step; rng <= total_range; rng += ring_step) {
            int radius_px = static_cast<int>(rng * scale);
            cv::ellipse(polar,
                        cv::Point(static_cast<int>(cx), static_cast<int>(cy)),
                        cv::Size(radius_px, radius_px),
                        0.0, arc_start, arc_end,
                        grid_col, 1, cv::LINE_AA);

            // Label at the top of each ring (along the centre axis)
            int label_y = static_cast<int>(cy) - radius_px - 3;
            if (label_y > 5 && label_y < S) {
                cv::putText(polar,
                            std::to_string(static_cast<int>(rng)) + "m",
                            cv::Point(static_cast<int>(cx) + 4, label_y),
                            cv::FONT_HERSHEY_PLAIN, 0.8, text_col, 1, cv::LINE_AA);
            }
        }

        // ---- Grid overlay: bearing lines every 10° ----
        for (int deg = -80; deg <= 80; deg += 10) {
            double angle_rad = deg * M_PI / 180.0;
            if (angle_rad < bear_min || angle_rad > bear_max) continue;
            int ex = static_cast<int>(cx + total_range * scale * std::sin(angle_rad));
            int ey = static_cast<int>(cy - total_range * scale * std::cos(angle_rad));
            cv::line(polar,
                     cv::Point(static_cast<int>(cx), static_cast<int>(cy)),
                     cv::Point(ex, ey),
                     grid_col, 1, cv::LINE_AA);
            // Degree label
            if (deg != 0) {
                cv::putText(polar, std::to_string(deg) + "\xc2\xb0",
                            cv::Point(ex, ey - 4),
                            cv::FONT_HERSHEY_PLAIN, 0.7, text_col, 1, cv::LINE_AA);
            }
        }

        return polar;
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
                // if (avg_distance <= filter_distance_threshold_) {
                filtered_cloud.push_back(focus_point);
                // }
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

        // ---- Pass 1: decode raw bytes into a normalised intensity grid ----
        // Layout: intensity_grid[r][b], matching the sonar's range × beam image.
        cv::Mat intensity_grid(n_ranges, n_beams, CV_32FC1);

        for (uint16_t r = 0; r < n_ranges; r++) {
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
                intensity_grid.at<float>(r, b) = static_cast<float>(raw_val / max_intensity);
            }
        }

        // ---- Median filter on the intensity image (optional) ----
        // Kernel size = 2R+1 per the paper; R=0 disables the filter.
        // cv::medianBlur requires CV_8U input, so we convert to 8-bit, blur, then back.
        cv::Mat intensity_grid_raw = intensity_grid.clone();

        if (median_filter_radius_ > 0) {
            int kernel_size = 2 * median_filter_radius_ + 1;
            cv::Mat grid_8u;
            intensity_grid.convertTo(grid_8u, CV_8U, 255.0);
            cv::medianBlur(grid_8u, grid_8u, kernel_size);
            grid_8u.convertTo(intensity_grid, CV_32F, 1.0 / 255.0);
        }

        // ---- Adaptive threshold (eq. 7-8): binarize the filtered grid ----
        // The result is a binary CV_32FC1 mask (1.0 = keep, 0.0 = reject).
        cv::Mat binary_grid = applyAdaptiveThreshold(intensity_grid);

        // ---- Publish polar images (raw and post-threshold), decimated ----
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

            publish_polar(intensity_grid_raw,  sonar_image_raw_pub_);
            publish_polar(binary_grid,          sonar_image_filt_pub_);
        }

        // ---- Pass 2: apply thresholds and project to 3-D ----
        std::vector<std::vector<pcl::PointXYZ>> points_by_beam(n_beams);

        for (uint16_t r = 0; r < n_ranges; r++) {
            double range = r * range_res;
            if (range < min_range_ || range > max_range_) continue;

            for (uint16_t b = 0; b < n_beams; b++) {
                // Adaptive threshold: skip pixels binarized to 0
                if (binary_grid.at<float>(r, b) < 0.5f) continue;

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
        pcl::PointCloud<pcl::PointXYZ> filtered_cloud = filterPointCloud(points_by_beam);

        // ---- Logging ----
        size_t total_after   = filtered_cloud.size();
        size_t total_removed = total_before - total_after;
        double pct = total_before > 0 ? 100.0 * total_removed / total_before : 0.0;
        RCLCPP_DEBUG(this->get_logger(),
                     "Filtering: %zu → %zu points (removed %zu, %.1f%%)",
                     total_before, total_after, total_removed, pct);

        // ---- Publish ----
        if (!filtered_cloud.empty()) {
            sensor_msgs::msg::PointCloud2 out_msg;
            pcl::toROSMsg(filtered_cloud, out_msg);
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