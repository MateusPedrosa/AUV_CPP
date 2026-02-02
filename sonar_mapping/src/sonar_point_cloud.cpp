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

        std::string sonar_topic = this->get_parameter("sonar_topic").as_string();
        std::string pose_topic = this->get_parameter("pose_topic").as_string();
        resolution_ = this->get_parameter("resolution").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        min_intensity_short_range_ = this->get_parameter("min_intensity_short_range").as_double();
        min_intensity_long_range_ = this->get_parameter("min_intensity_long_range").as_double();
        h_fov_ = this->get_parameter("horizontal_fov").as_double() * M_PI / 180.0;
        v_fov_ = this->get_parameter("vertical_fov").as_double() * M_PI / 180.0;


        // Create transformation matrix from sonar_link to base_link
        // Translation: (0.3, 0.0, 0.3)
        // Rotation: Roll=0°, Pitch=-45°, Yaw=-90°
        createSonarToBaseLinkTransform(0.3, 0.0, 0.3, 0.0, 45.0, 0.0);

        // TODO: check qos
        rclcpp::QoS qos_profile(1);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sonar_point_cloud", qos_profile);

        rclcpp::QoS qos(10);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
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
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    Eigen::Matrix4f sonar_to_baselink_transform_;
    double resolution_;
    double max_range_;
    double min_intensity_short_range_;
    double min_intensity_long_range_;
    double h_fov_;
    double v_fov_;

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

        RCLCPP_INFO(this->get_logger(), "Processing sonar image: %dx%d", cols, rows);

        // Clear previous cloud
        cloud_.clear();
        cloud_.header.frame_id = "sonar_link";

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

                // Convert to 3D Cartesian coordinates (equation 3.7)
                // x = r * cos(theta)  - forward direction
                // y = r * sin(theta)  - lateral direction
                // z = depth           - robot's current depth (constant for this ping)

                // Frame: X=Forward, Y=Left, Z=Up
                double x = range * cos(bearing);
                double y = range * sin(bearing);
                double z = 0.0; // Assuming flat plane at sonar depth

                // Calculate in "optical" frame
                // Frame: X=Right, Y=Up, -Z=Forward
                // double x_opt = range * sin(bearing);   // Right
                // double y_opt = 0.0;                    // Up
                // double z_opt = -range * cos(bearing);  // Forward is -Z


                // double x = -z_opt; // Forward
                // double y = -x_opt; // Left = -Right
                // double z = y_opt;  // Up = Up

                // Transform to "sonar_link" frame
                // Frame: X=Right, Y=Up, -Z=Forward
                // double x = y_opt;
                // double y = z_opt;
                // double z = -x_opt;

                // Apply voxel grid filtering (simple decimation)
                int voxel_x = static_cast<int>(x / resolution_);
                int voxel_y = static_cast<int>(y / resolution_);
                int voxel_z = static_cast<int>(z / resolution_);

                // Snap to voxel center
                pcl::PointXYZ point;
                point.x = (voxel_x + 0.5) * resolution_;
                point.y = (voxel_y + 0.5) * resolution_;
                point.z = (voxel_z + 0.5) * resolution_;

                cloud_.push_back(point);
            }
        }

        // Publish point cloud with same timestamp as input image
        if (!cloud_.empty()) {
            pcl::PointCloud<pcl::PointXYZ> cloud_baselink;
            pcl::transformPointCloud(cloud_, cloud_baselink, sonar_to_baselink_transform_);
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(cloud_baselink, output_msg);
            output_msg.header.frame_id = "base_link";
            output_msg.header.stamp = img_msg->header.stamp; // Use original timestamp

            point_cloud_pub_->publish(output_msg);
            RCLCPP_INFO(this->get_logger(), "Published point cloud with %zu points", cloud_.size());
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