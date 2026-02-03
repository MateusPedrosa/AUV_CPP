#include "rclcpp/rclcpp.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// #include "message_filters/subscriber.hpp"
// #include "message_filters/synchronizer.hpp"
// #include "message_filters/sync_policies/approximate_time.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
// using std::placeholders::_2;

// TODO: use OculusPing message type

class SonarPointCloud : public rclcpp::Node
{
public:
    SonarPointCloud() : Node("sonar_point_cloud") {
        // Parameters
        this->declare_parameter("sonar_topic", "/oceansim/robot/imaging_sonar");
        this->declare_parameter("pose_topic", "/oceansim/robot/pose");
        this->declare_parameter("resolution", 0.1); // 10cm voxels
        this->declare_parameter("max_range", 3.0); // Maximum sonar range in meters
        this->declare_parameter("min_intensity_short_range", 0.4); // Minimum intensity threshold for short range
        this->declare_parameter("min_intensity_long_range", 0.2); // Minimum intensity threshold for long range
        this->declare_parameter("horizontal_fov", 130.0); // Horizontal field of view in degrees
        this->declare_parameter("vertical_fov", 20.0); // Vertical field of view in degrees
        this->declare_parameter("filter_window_size", 7); // Window size s (must be even, s+1 total beams)
        this->declare_parameter("filter_distance_threshold", 0.5); // Max average distance threshold in meters

        std::string sonar_topic = this->get_parameter("sonar_topic").as_string();
        std::string pose_topic = this->get_parameter("pose_topic").as_string();
        resolution_ = this->get_parameter("resolution").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        min_intensity_short_range_ = this->get_parameter("min_intensity_short_range").as_double();
        min_intensity_long_range_ = this->get_parameter("min_intensity_long_range").as_double();
        h_fov_ = this->get_parameter("horizontal_fov").as_double() * M_PI / 180.0;
        v_fov_ = this->get_parameter("vertical_fov").as_double() * M_PI / 180.0;
        filter_window_size_ = this->get_parameter("filter_window_size").as_int();
        filter_distance_threshold_ = this->get_parameter("filter_distance_threshold").as_double();

        // Ensure window size is even
        if (filter_window_size_ % 2 != 0) {
            RCLCPP_WARN(this->get_logger(), "Filter window size must be even, adjusting %d to %d",
                        filter_window_size_, filter_window_size_ + 1);
            filter_window_size_++;
        }

        // Create transformation matrix from sonar_link to base_link
        // Translation: (0.3, 0.0, 0.3)
        // Rotation: Roll=0°, Pitch=-45°, Yaw=-90°
        createSonarToBaseLinkTransform(0.3, 0.0, 0.3, 0.0, 45.0, 0.0);

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
        // sonar_subscriber_.subscribe(this, sonar_topic, qos.get_rmw_qos_profile());
        // pose_subscriber_.subscribe(this, pose_topic, qos.get_rmw_qos_profile());

        // uint32_t queue_size = 10;
        // sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::
        //     ApproximateTime<sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped>>>(
        //     message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
        //     geometry_msgs::msg::PoseStamped>(queue_size), sonar_subscriber_, pose_subscriber_);

        // sync->setAgePenalty(0.50); // TODO: tune this value
        // sync->registerCallback(std::bind(&SonarPointCloud::SyncCallback, this, _1, _2));
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sonar_subscriber_;
    // message_filters::Subscriber<sensor_msgs::msg::Image> sonar_subscriber_;
    // message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_subscriber_;
    // std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
    //     sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped>>> sync;
    Eigen::Matrix4f sonar_to_baselink_transform_;
    double resolution_;
    double max_range_;
    double min_intensity_short_range_;
    double min_intensity_long_range_;
    double h_fov_;
    double v_fov_;
    int filter_window_size_;
    double filter_distance_threshold_;

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
                        // Calculate Euclidean distance (equation 54)
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

    void createSonarToBaseLinkTransform(double tx, double ty, double tz,
                                        double roll_deg, double pitch_deg, double yaw_deg) {
        // Convert degrees to radians
        double roll = roll_deg * M_PI / 180.0;
        double pitch = pitch_deg * M_PI / 180.0;
        double yaw = yaw_deg * M_PI / 180.0;

        // Create rotation matrix using Eigen (ZYX euler angles - ROS convention)
        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
        Eigen::Matrix3f rotation = q.matrix();

        // Create 4x4 transformation matrix
        sonar_to_baselink_transform_ = Eigen::Matrix4f::Identity();
        sonar_to_baselink_transform_.block<3,3>(0,0) = rotation;
        sonar_to_baselink_transform_(0,3) = tx;
        sonar_to_baselink_transform_(1,3) = ty;
        sonar_to_baselink_transform_(2,3) = tz;

        RCLCPP_INFO(this->get_logger(), "Sonar to base_link transform created:");
        RCLCPP_INFO(this->get_logger(), "Translation: [%.2f, %.2f, %.2f]", tx, ty, tz);
        RCLCPP_INFO(this->get_logger(), "Rotation (RPY): [%.2f, %.2f, %.2f] deg", roll_deg, pitch_deg, yaw_deg);
    }

    // void SyncCallback(const sensor_msgs::msg::Image::ConstSharedPtr & sonar_data,
    //                   const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_data) {
    //     RCLCPP_INFO(this->get_logger(), "Sync callback with %u and %u as times",
    //     sonar_data->header.stamp.sec, pose_data->header.stamp.sec);

    //     convertImageToPointCloud(sonar_data);
    // }

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

        RCLCPP_DEBUG(this->get_logger(), "Processing sonar image: %dx%d", cols, rows);

        std::vector<std::vector<pcl::PointXYZ>> points_by_beam(cols);

        // M750d produces 2D range-bearing data at constant depth
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                float intensity = sonar_image.at<float>(r, c);

                // Calculate range (distance from sonar)
                double range = (static_cast<double>(r) / rows) * max_range_;

                // Filter by minimum intensity thresholds
                if ((range < 3.0 && intensity < min_intensity_short_range_) ||
                    (range >= 3.0 && intensity < min_intensity_long_range_)) {
                    continue;
                }

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

        pcl::PointCloud<pcl::PointXYZ> cloud = filterPointCloud(points_by_beam);
        cloud.header.frame_id = "sonar_link";

        RCLCPP_DEBUG(this->get_logger(), "Filtered cloud from %zu beams to %zu points", points_by_beam.size(), cloud.size());

        size_t points_after = cloud.size();
        size_t points_removed = total_points_before - points_after;
        double filter_percentage = total_points_before > 0 ? 
            (100.0 * points_removed / total_points_before) : 0.0;

        RCLCPP_DEBUG(this->get_logger(), 
                    "Filtering: %zu points before -> %zu points after (removed %zu points, %.1f%%)",
                    total_points_before, points_after, points_removed, filter_percentage);

        // Publish point cloud with same timestamp as input image
        if (!cloud.empty()) {
            pcl::PointCloud<pcl::PointXYZ> cloud_baselink;
            pcl::transformPointCloud(cloud, cloud_baselink, sonar_to_baselink_transform_);
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(cloud_baselink, output_msg);
            output_msg.header.frame_id = "base_link";
            output_msg.header.stamp = this->now(); //img_msg->header.stamp;

            point_cloud_pub_->publish(output_msg);
            RCLCPP_DEBUG(this->get_logger(), "Published point cloud with %zu points", cloud.size());
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