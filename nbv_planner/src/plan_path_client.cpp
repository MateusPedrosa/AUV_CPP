#include <memory>
#include <chrono>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nbv_planner/action/plan_path.hpp"

using namespace std::chrono_literals;

class PlanPathClient : public rclcpp::Node
{
public:
    using PlanPath = nbv_planner::action::PlanPath;
    using GoalHandlePlanPath = rclcpp_action::ClientGoalHandle<PlanPath>;

    explicit PlanPathClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("plan_path_client", options),
          received_pose_(false)
    {
        this->client_ = rclcpp_action::create_client<PlanPath>(this, "plan_path");
        
        // Subscribe to robot pose
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/oceansim/robot/pose",
            10,
            std::bind(&PlanPathClient::poseCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Waiting for robot pose on /oceansim/robot/pose...");

        this->declare_parameter("goal_x", 5.0);
        this->declare_parameter("goal_y", 5.0);
        this->declare_parameter("goal_z", 1.0);
        this->declare_parameter("goal_roll", 0.0);
        this->declare_parameter("goal_pitch", 0.0);
        this->declare_parameter("goal_yaw", 0.0);
        this->declare_parameter("time_limit", 1.0);
    }

    void send_goal()
    {
        // Wait for robot pose
        while (!received_pose_ && rclcpp::ok()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for robot pose...");
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }
        
        if (!received_pose_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive robot pose");
            rclcpp::shutdown();
            return;
        }
        
        if (!this->client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = PlanPath::Goal();
        
        // Use latest robot pose as start pose
        goal_msg.start_pose = latest_pose_.pose;
        
        // Set goal pose from parameters
        goal_msg.goal_pose.position.x = this->get_parameter("goal_x").as_double();
        goal_msg.goal_pose.position.y = this->get_parameter("goal_y").as_double();
        goal_msg.goal_pose.position.z = this->get_parameter("goal_z").as_double();

        // Get Euler angles in degrees from parameters
        double roll_deg = this->get_parameter("goal_roll").as_double();
        double pitch_deg = this->get_parameter("goal_pitch").as_double();
        double yaw_deg = this->get_parameter("goal_yaw").as_double();

        // Convert Euler to TF2 Quaternion
        tf2::Quaternion q;
        q.setRPY(deg2rad(roll_deg), deg2rad(pitch_deg), deg2rad(yaw_deg));

        // Convert TF2 Quaternion to Geometry Message Quaternion
        goal_msg.goal_pose.orientation = tf2::toMsg(q);

        goal_msg.time_limit = this->get_parameter("time_limit").as_double();

        RCLCPP_INFO(this->get_logger(), "Sending goal from current pose (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
                    goal_msg.start_pose.position.x,
                    goal_msg.start_pose.position.y,
                    goal_msg.start_pose.position.z,
                    goal_msg.goal_pose.position.x,
                    goal_msg.goal_pose.position.y,
                    goal_msg.goal_pose.position.z);

        auto send_goal_options = rclcpp_action::Client<PlanPath>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            std::bind(&PlanPathClient::goal_response_callback, this, std::placeholders::_1);
        
        send_goal_options.feedback_callback =
            std::bind(&PlanPathClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        
        send_goal_options.result_callback =
            std::bind(&PlanPathClient::result_callback, this, std::placeholders::_1);

        this->client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<PlanPath>::SharedPtr client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    geometry_msgs::msg::PoseStamped latest_pose_;
    bool received_pose_;
    
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        latest_pose_ = *msg;
        if (!received_pose_) {
            RCLCPP_INFO(this->get_logger(), "Received robot pose: (%.2f, %.2f, %.2f)",
                        msg->pose.position.x,
                        msg->pose.position.y,
                        msg->pose.position.z);
            received_pose_ = true;
        }
    }

    void goal_response_callback(const GoalHandlePlanPath::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandlePlanPath::SharedPtr,
        const std::shared_ptr<const PlanPath::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Feedback: %s (Progress: %.2f%%)", 
                    feedback->status.c_str(), feedback->progress * 100.0);
    }

    void result_callback(const GoalHandlePlanPath::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                RCLCPP_INFO(this->get_logger(), "Result: %s", result.result->message.c_str());
                RCLCPP_INFO(this->get_logger(), "Path contains %zu poses", result.result->path.poses.size());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                RCLCPP_ERROR(this->get_logger(), "Result: %s", result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
        rclcpp::shutdown();
    }

    double deg2rad(double degrees) {
        return degrees * (M_PI / 180.0);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto action_client = std::make_shared<PlanPathClient>();
    action_client->send_goal();
    
    rclcpp::spin(action_client);
    
    rclcpp::shutdown();
    return 0;
}