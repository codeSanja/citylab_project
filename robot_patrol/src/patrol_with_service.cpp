#include "rclcpp/logging.hpp"
#include <chrono>
#include <fstream>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <thread>
#include <std_srvs/srv/trigger.hpp>
#include "robot_patrol/srv/get_direction.hpp"

struct ObstacleData {
    std::map<std::string, float> distances;
    std::map<std::string, bool> detections;
};

class PatrolWithService : public rclcpp::Node {
public:
    PatrolWithService() : Node("patrol_node") {
        is_construct_environment_ = true;
        std::string laser_subscriber_name =
                is_construct_environment_ ? "/fastbot_1/scan" : "/vehicle_green/scan";
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser_subscriber_name, 10,
            std::bind(&PatrolWithService::laser_callback, this, std::placeholders::_1));

        std::string twist_publisher_name = is_construct_environment_
                                               ? "/fastbot_1/cmd_vel"
                                               : "/model/vehicle_green/cmd_vel";
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            twist_publisher_name, 10);

        control_timer_ =
                this->create_wall_timer(std::chrono::milliseconds(100),
                                        std::bind(&PatrolWithService::send_request, this));

        std::string name_service = "/direction_service";
        client_ = this->create_client<robot_patrol::srv::GetDirection>(name_service);

        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the "
                             "service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service %s not available, waiting again...", name_service.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Client Ready");
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        current_laser_scan_ = msg;
    }

    void send_request() {
        if (!current_laser_scan_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for laser scan...");
            return;
        }
        auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
        request->laser_data = *current_laser_scan_;
        auto result_future = client_->async_send_request(request,
             [this](
         rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture
         future) {
                 auto response = future.get();

                 RCLCPP_INFO(this->get_logger(), "Response received");

                 RCLCPP_INFO(
                     this->get_logger(),
                     "Direction decision: %s",
                     response->direction.c_str()
                 );

                 auto action = geometry_msgs::msg::Twist();

                 if (response->direction == "forward") {
                     action.linear.x =
                             is_construct_environment_ ? 0.1 : 0.9;
                     action.angular.z = 0;
                 } else if (response->direction == "left") {
                     action.linear.x = is_construct_environment_
                             ? 0.02
                             : 0.3;
                     action.angular.z = 0.5;
                 } else if (response->direction == "right") {
                     action.linear.x = is_construct_environment_
                             ? 0.02
                             : 0.3;
                     action.angular.z = -0.5;
                 }

                 twist_publisher_->publish(action);
             }
        );

        RCLCPP_INFO(this->get_logger(), "Request Sent");
    }

private:
    bool is_construct_environment_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
    laser_subscriber_;
    sensor_msgs::msg::LaserScan::SharedPtr current_laser_scan_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto client = std::make_shared<PatrolWithService>();

    rclcpp::spin(client);

    rclcpp::shutdown();

    return 0;
}
