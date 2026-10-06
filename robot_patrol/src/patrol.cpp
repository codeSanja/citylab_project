#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
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

struct ObstacleData {
    std::map<std::string, float> distances;
    std::map<std::string, bool> detections;

    float safest_distance = 0.0;
    int safest_index = -1;
};

class Patrol : public rclcpp::Node {
public:
    Patrol() : Node("patrol_node") {

        std::string laser_subscriber_name = is_construct_environment_ ? "/laser_scan": "/vehicle_green/scan";
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_subscriber_name, 10, std::bind(&Patrol::laser_callback, this, std::placeholders::_1));

        std::string twist_publisher_name = is_construct_environment_ ? "/fastbot_1/cmd_vel" : "/model/vehicle_green/cmd_vel";
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(twist_publisher_name, 10);

        control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Patrol::patrol_around, this)
    );

    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        current_laser_scan_ = msg;
    }

    void patrol_around() {
        if (!current_laser_scan_) {
            RCLCPP_WARN(
                this->get_logger(),
                "Waiting for laser scan..."
            );
            return;
        }

        auto action = geometry_msgs::msg::Twist();
        ObstacleData obstacle_data = get_obstacle_data();
        bool is_obsticle_in_front = obstacle_data.detections["Front_Left"] || obstacle_data.detections["Front_Right"];

        if (is_obsticle_in_front) {
            action.linear.x = is_construct_environment_ ? 0.05 : 0.3;
            int center_index = current_laser_scan_->ranges.size() / 2;

            action.angular.z = obstacle_data.safest_index < center_index ? -0.5 : 0.5;
        } else {
            action.linear.x = is_construct_environment_ ? 0.1 : 0.9;
            action.angular.z = 0.0;
        }

        twist_publisher_->publish(action);


    }

    ObstacleData get_obstacle_data()
    {
        ObstacleData result;

        std::map<std::string, std::pair<int, int>> sectors;

        if (is_construct_environment_) {
            sectors = {
                {"Right_Rear",  {0, 33}},
                {"Right",       {34, 66}},
                {"Front_Right", {67, 100}},
                {"Front_Left",  {101, 133}},
                {"Left",        {134, 166}},
                {"Left_Rear",   {167, 199}}
            };
        } else {
            sectors = {
                {"Right_Rear",  {0, 59}},
                {"Right",       {60, 149}},
                {"Front_Right", {150, 179}},
                {"Front_Left",  {180, 209}},
                {"Left",        {210, 299}},
                {"Left_Rear",   {300, 359}}
            };
        }

        double obstacle_threshold =
            is_construct_environment_ ? 0.35 : 2.4;

        // min_distance
        for (const auto& sector : sectors) {

            int start_idx = sector.second.first;
            int end_idx = sector.second.second;

            float min_distance =
                std::numeric_limits<float>::infinity();

            if (
                start_idx <
                    static_cast<int>(current_laser_scan_->ranges.size()) &&
                end_idx <
                    static_cast<int>(current_laser_scan_->ranges.size())
            ) {
                auto start_it =
                    current_laser_scan_->ranges.begin() + start_idx;

                auto end_it =
                    current_laser_scan_->ranges.begin() + end_idx + 1;

                min_distance =
                    *std::min_element(start_it, end_it);
            }

            // Save actual distance
            result.distances[sector.first] = min_distance;

            // Save detection
            result.detections[sector.first] =
                min_distance < obstacle_threshold;
        }

        float greatest_distance = 0.0;
        int greatest_distance_index = -1;

        for (int i = 0;
             i < static_cast<int>(current_laser_scan_->ranges.size());
             ++i)
        {
            float distance = current_laser_scan_->ranges[i];

            bool valid =
                std::isfinite(distance) &&
                distance >= current_laser_scan_->range_min &&
                distance <= current_laser_scan_->range_max;

            if (valid && distance > greatest_distance) {
                greatest_distance = distance;
                greatest_distance_index = i;
            }
        }

        result.safest_distance = greatest_distance; // Maybe needed for debugging
        result.safest_index = greatest_distance_index;

        return result;
    }


private:
    bool is_construct_environment_ = false; // TODO flip for the Construct environment
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    sensor_msgs::msg::LaserScan::SharedPtr current_laser_scan_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto patrol_node = std::make_shared<Patrol>();
    // auto green_detector_node = std::make_shared<GreenDetecotNode>(plant_detector_node);

    // Use MultiThreadedExecutor with 2 threads
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                      2);
    executor.add_node(patrol_node);
    // executor.add_node(green_detector_node);

    try {
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(patrol_node->get_logger(), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}