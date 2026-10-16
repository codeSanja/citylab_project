#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include "robot_patrol/action/go_to_pose.hpp"
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class GoToPoseActionServer : public rclcpp::Node {
public:
    using GoToPose = robot_patrol::action::GoToPose;
    using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;

    GoToPoseActionServer() : Node("go_to_pose_action") {
        is_construct_environment_ = false; // TODO Update for the Construct environment
        is_construct_simulation_ = false; // TODO Update for the Construct environment

        using namespace std::placeholders;

        // Action server to accept goals
        this->action_server_ = rclcpp_action::create_server<GoToPose>(
            this, "go_to_pose",
            std::bind(&GoToPoseActionServer::handle_goal, this, _1, _2),
            std::bind(&GoToPoseActionServer::handle_cancel, this, _1),
            std::bind(&GoToPoseActionServer::handle_accepted, this, _1));


        std::string odom_subscriber_name = is_construct_environment_
                                               ? (is_construct_simulation_ ? "/fastbot_1/odom" : "/odom")
                                               : "/model/vehicle_green/odometry";

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_subscriber_name,
            10,
            std::bind(&GoToPoseActionServer::odomCallback, this, _1)
        );

        std::string twist_publisher_name = is_construct_environment_
                                               ? (is_construct_simulation_ ? "/fastbot_1/cmd_vel" : "/cmd_vel")
                                               : "/model/vehicle_green/cmd_vel";

        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            twist_publisher_name,
            10);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto &orientation = msg->pose.pose.orientation;
        tf2::Quaternion q(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        );
        double roll, pitch, theta;
        tf2::Matrix3x3(q).getRPY(roll, pitch, theta);

        current_pos_.x = msg->pose.pose.position.x;
        current_pos_.y = msg->pose.pose.position.y;
        current_pos_.theta = theta;

        RCLCPP_INFO(
            this->get_logger(),
            "ODOM CALLBACK: x=%.3f, y=%.3f, theta=%.3f",
            current_pos_.x,
            current_pos_.y,
            current_pos_.theta
        );
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const GoToPose::Goal> goal) {
        RCLCPP_INFO(this->get_logger(),
                    "Received goal request with x: %.2f, y: %.2f, yaw: %.2f",
                    goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
        (void) uuid;

        desired_pos_ = goal->goal_pos;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void) goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        using namespace std::placeholders;

        std::thread{std::bind(&GoToPoseActionServer::execute, this, _1), goal_handle}
                .detach();
    }

    void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<GoToPose::Feedback>();
        auto result = std::make_shared<GoToPose::Result>();

        feedback->current_pos.x = goal->goal_pos.x;
        feedback->current_pos.y = goal->goal_pos.y;
        feedback->current_pos.theta = goal->goal_pos.theta;
        goal_handle->publish_feedback(feedback);

        bool navigation_result = navigate_to_goal(goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);

        if (navigation_result) {
            RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
            result->status = true;
            goal_handle->succeed(result);
        } else {
            RCLCPP_INFO(this->get_logger(), "Failed to reach goal :(");
            result->status = false;
            goal_handle->abort(result);
        }
    }

    bool navigate_to_goal(double x, double y, double theta) {
        rclcpp::Rate rate(10);

        RCLCPP_INFO(
            this->get_logger(),
            "Navigating to x: %.2f, y: %.2f, theta: %.2f",
            x, y, theta
        );

        double position_tolerance = is_construct_environment_ ? 0.075 : 0.09;
        double angle_tolerance = is_construct_environment_ ? 0.1745 : 0.23;

        while (rclcpp::ok()) {
            double dx = x - current_pos_.x;
            double dy = y - current_pos_.y;
            double current_theta = current_pos_.theta;
            double distance = std::hypot(dx, dy);
            double goal_direction = std::atan2(dy, dx);

            double angular_error = goal_direction - current_theta;
            angular_error = std::atan2(
                std::sin(angular_error),
                std::cos(angular_error)
            );

            RCLCPP_INFO(
                this->get_logger(),
                "Distance: %.2f, Current theta: %.2f, Goal direction: %.2f, Angular error: %.2f",
                distance, current_theta, goal_direction, angular_error
            );

            geometry_msgs::msg::Twist cmd;

            if (distance > position_tolerance) {
                if (std::abs(angular_error) > angle_tolerance) {
                    cmd.angular.z = angular_error > 0 ? 0.5 : -0.5;
                } else {
                    cmd.linear.x = is_construct_environment_ ? 0.2 : 0.3;
                }
            } else {
                double final_theta = theta - current_theta;
                final_theta = std::atan2(
                    std::sin(final_theta),
                    std::cos(final_theta)
                );

                if (std::abs(final_theta) > angle_tolerance) {
                    cmd.angular.z = final_theta > 0 ? 0.5 : -0.5;
                } else {
                    // Position and orientation reached!
                    twist_publisher_->publish(geometry_msgs::msg::Twist());
                    return true;
                }
            }

            twist_publisher_->publish(cmd);
            rate.sleep();
        }

        // Stop if ROS shuts down
        twist_publisher_->publish(geometry_msgs::msg::Twist());
        return false;
    }

    bool is_construct_environment_;
    bool is_construct_simulation_;
    rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
    geometry_msgs::msg::Pose2D desired_pos_;
    geometry_msgs::msg::Pose2D current_pos_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToPoseActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
