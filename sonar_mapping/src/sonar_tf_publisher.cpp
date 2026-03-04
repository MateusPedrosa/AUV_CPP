#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "blueye_interfaces/msg/float_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>

class SonarTfPublisher : public rclcpp::Node
{
public:
    SonarTfPublisher() : Node("sonar_tf_publisher"), current_pitch_(0.0)
    {
        this->declare_parameter("pitch_topic", "/blueye/sensor/sonar/servo_angle");
        this->declare_parameter("sonar_frame", "sonar_link");
        this->declare_parameter("robot_frame", "base_link");
        this->declare_parameter("translation_x", 0.1);
        this->declare_parameter("translation_y", 0.0);
        this->declare_parameter("translation_z", 0.2);
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        rclcpp::QoS qos(1);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        pitch_sub_ = this->create_subscription<blueye_interfaces::msg::FloatStamped>(
            this->get_parameter("pitch_topic").as_string(), qos,
            std::bind(&SonarTfPublisher::pitchCallback, this, std::placeholders::_1)
        );

        // Publish at 50 Hz so TF never goes stale, even if the servo is slow
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&SonarTfPublisher::publishTransform, this)
        );
    }

private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<blueye_interfaces::msg::FloatStamped>::SharedPtr pitch_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double current_pitch_;


    void pitchCallback(const blueye_interfaces::msg::FloatStamped::SharedPtr msg)
    {
        current_pitch_ = msg->data * M_PI / 180.0;
    }

    void publishTransform()
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = this->get_parameter("robot_frame").as_string();
        t.child_frame_id = this->get_parameter("sonar_frame").as_string();

        t.transform.translation.x = this->get_parameter("translation_x").as_double();
        t.transform.translation.y = this->get_parameter("translation_y").as_double();
        t.transform.translation.z = this->get_parameter("translation_z").as_double();

        tf2::Quaternion q;
        q.setRPY(0.0, current_pitch_, 0.0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SonarTfPublisher>());
    rclcpp::shutdown();
    return 0;
}