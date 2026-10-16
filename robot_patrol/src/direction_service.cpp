#include "rclcpp/logging.hpp"
#include <chrono>
#include <fstream>
// #include <geometry_msgs/msg/twist.hpp>
#include <memory>
// #include <opencv2/opencv.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <thread>
#include <std_srvs/srv/trigger.hpp>
#include <robot_patrol/srv/get_direction.hpp>
#include <cmath>

class DirectionService : public rclcpp::Node {
public:
    DirectionService() : Node("direction_service") {
        is_construct_environment_ = true;
        std::string name_service = "/direction_service";
        service_ = this->create_service<robot_patrol::srv::GetDirection>(
            name_service,
            std::bind(&DirectionService::get_direction_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "%s - Service Server Ready", name_service.c_str());
    }

private:
    void get_direction_callback(
        const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
        std::shared_ptr<robot_patrol::srv::GetDirection::Response> response) {

        RCLCPP_INFO(this->get_logger(), "Request Received");

        auto laser_data = request->laser_data;
        double obstacle_threshold = is_construct_environment_ ? 0.35 : 3.0;

        size_t right_begin;
        size_t right_end;
        size_t front_begin;
        size_t front_end;
        size_t left_begin;
        size_t left_end;

        if (is_construct_environment_) {
            bool is_construct_simulation = true; // TODO Update in the real-robot branch

            if (is_construct_simulation) {
                // Construct simulation: 200 rays
                right_begin = 134;
                right_end = 188;

                front_begin = 188;
                front_end = 12;  // Wraparound

                left_begin = 12;
                left_end = 67;
            } else { // Construct real robot
                right_begin = 303;
                right_end = 375;

                front_begin = 375;
                front_end = 77;

                left_begin = 77;
                left_end = 151;
            }

        } else { // local gazebo
            right_begin = 90;
            right_end = 150;

            front_begin = 150;
            front_end = 210;

            left_begin = 210;
            left_end = 270;
        }


        double min_forward_distance = laser_data.ranges[front_begin+1];

        RCLCPP_INFO(
            this->get_logger(),
            "LaserScan: size=%zu, angle_min=%.3f, angle_max=%.3f, increment=%.5f",
            laser_data.ranges.size(),
            laser_data.angle_min,
            laser_data.angle_max,
            laser_data.angle_increment
        );

        for (size_t i = 0; i < laser_data.ranges.size(); i++) {
            RCLCPP_INFO(
                this->get_logger(),
                "LaserScan[%zu] = %.3f m",
                i,
                laser_data.ranges[i]
            );
        }

        // make decisions based on laser_data
        float total_dist_sec_right = 0.0;
        float total_dist_sec_front = 0.0;
        float total_dist_sec_left = 0.0;

        for (size_t i = 0; i < laser_data.ranges.size(); i++) {
            float distance = laser_data.ranges[i];
            if (std::isinf(distance) && distance > 0) continue;


            bool is_front = false;
            if(is_construct_environment_) {
                is_front = front_begin < front_end
                    ? (i >= front_begin && i < front_end)
                    : (i >= front_begin || i < front_end);
            } else {
                is_front = i >= front_begin && i < front_end;
            }

            if (i >= right_begin && i < right_end) {
                total_dist_sec_right += distance;
            } else if (is_front) {
                total_dist_sec_front += distance; // total_dist_sec_front - not used, but kept for debugging
                if (min_forward_distance > distance) { min_forward_distance = distance; }
            } else if (i >= left_begin && i < left_end) {
                total_dist_sec_left += distance;
            }
        }

        if (min_forward_distance > obstacle_threshold) {
            response->direction = "forward";
        } else if (total_dist_sec_left >= total_dist_sec_right) {
            response->direction = "left";
        } else {
            response->direction = "right";
        }


        RCLCPP_INFO(
            this->get_logger(),
            "Right: %.2f | Front: %.2f | Left: %.2f | **Direction** : %s",
            total_dist_sec_right,
            total_dist_sec_front,
            total_dist_sec_left,
            response->direction.c_str()
        );

        RCLCPP_INFO(this->get_logger(), "Request Completed");
    }

    rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_;
    bool is_construct_environment_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DirectionService>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
