#include "rclcpp/rclcpp.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>

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
    pcl::PointCloud<pcl::PointXYZ> filterPointCloud(
        const std::vector<std::vector<pcl::PointXYZ>>& points_by_beam) {
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

        std::vector<std::vector<pcl::PointXYZ>> points_by_beam(cols);

        // M750d produces 2D range-bearing data at constant depth
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
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

                // Frame: X=Forward, Y=Left, Z=Up
                double x = range * cos(bearing);
                double y = range * sin(bearing);
                double z = 0.0; // Assuming flat plane at sonar depth

                pcl::PointXYZ point;
                point.x = x;
                point.y = y;
                point.z = z;

                points_by_beam[c].push_back(point);
            }
        }

        // Count total points before filtering
        size_t total_points_before = 0;
        for (const auto& beam : points_by_beam) {
            total_points_before += beam.size();
        }

        pcl::PointCloud<pcl::PointXYZ> window_filtered_cloud = filterPointCloud(points_by_beam);

        pcl::PointCloud<pcl::PointXYZ>::Ptr window_filtered_cloud_ptr = window_filtered_cloud.makeShared();
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud = applyRadiusFilter(window_filtered_cloud_ptr);

        final_cloud->header.frame_id = "sonar_link";

        RCLCPP_DEBUG(this->get_logger(), "Filtered cloud from %zu beams to %zu points", points_by_beam.size(), final_cloud->size());

        size_t points_after = final_cloud->size();
        size_t points_removed = total_points_before - points_after;
        double filter_percentage = total_points_before > 0 ? 
            (100.0 * points_removed / total_points_before) : 0.0;

        RCLCPP_DEBUG(this->get_logger(), 
                    "Filtering: %zu points before -> %zu points after (removed %zu points, %.1f%%)",
                    total_points_before, points_after, points_removed, filter_percentage);

        // Publish filtered point cloud
        if (!final_cloud->empty()) {
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(*final_cloud, output_msg);
            output_msg.header.frame_id = "sonar_link";
            output_msg.header.stamp = this->now();

            point_cloud_pub_->publish(output_msg);
            RCLCPP_DEBUG(this->get_logger(), "Published point cloud with %zu points", final_cloud->size());
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