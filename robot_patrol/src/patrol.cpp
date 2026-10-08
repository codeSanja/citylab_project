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

struct ObstacleData {
  std::map<std::string, float> distances;
  std::map<std::string, bool> detections;
};

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {

    is_construct_environment_ = true;
    std::string laser_subscriber_name =
        is_construct_environment_ ? "/scan" : "/vehicle_green/scan";
    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        laser_subscriber_name, 10,
        std::bind(&Patrol::laser_callback, this, std::placeholders::_1));

    std::string twist_publisher_name = is_construct_environment_
                                           ? "/cmd_vel"
                                           : "/model/vehicle_green/cmd_vel";
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        twist_publisher_name, 10);

    control_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&Patrol::patrol_around, this));
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    current_laser_scan_ = msg;
  }

  void patrol_around() {
    if (!current_laser_scan_) {
      RCLCPP_WARN(this->get_logger(), "Waiting for laser scan...");
      return;
    }

    auto action = geometry_msgs::msg::Twist();
    ObstacleData obstacle_data = get_obstacle_data();

    // move forward if there is no obsticle in front
    if (obstacle_data.detections["Front_Front_Left"] ||
        obstacle_data.detections["Front_Front_Right"]) {
      float left_clearance = std::min(obstacle_data.distances["Front_Left"],
                                      obstacle_data.distances["Left"]);

      float right_clearance = std::min(obstacle_data.distances["Front_Right"],
                                       obstacle_data.distances["Right"]);

        RCLCPP_INFO(
            this->get_logger(),
            "AVOIDANCE :: \n"
            "Front_Left=%.3f | Left=%.3f | \n"
            "Front_Right=%.3f | Right=%.3f || "
            "left_clearance=%.3f | right_clearance=%.3f || \n"
            "TURN=%s \n\n",
            obstacle_data.distances["Front_Left"],
            obstacle_data.distances["Left"],
            obstacle_data.distances["Front_Right"],
            obstacle_data.distances["Right"],
            left_clearance,
            right_clearance,
            left_clearance >= right_clearance ? "LEFT" : "RIGHT"
        );


      // get the furthest distance and turn toward there
      action.linear.x = is_construct_environment_ ? 0.05 : 0.3;
      action.angular.z = left_clearance >= right_clearance ? 0.5 : -0.5;

    } else {
      // move forward
      action.linear.x = is_construct_environment_ ? 0.1 : 0.9;
      action.angular.z = 0.0;
    }

    twist_publisher_->publish(action);
  }

  ObstacleData get_obstacle_data() {
    ObstacleData result;

    std::map<std::string, std::pair<int, int>> sectors;

    if (is_construct_environment_) {
    //   sectors = {
    //       {"Front_Front_Left", {0, 11}},
    //       {"Front_Left", {12, 33}},
    //       {"Left", {34, 66}},
    //       {"Left_Rear", {67, 99}},

    //       {"Right_Rear", {100, 133}},
    //       {"Right", {134, 165}},
    //       {"Front_Right", {166, 187}},
    //       {"Front_Front_Right", {188, 199}},
    //   };

        // sectors = {
        //     {"Front_Front_Left", {0, 11}},
        //     {"Front_Left", {12, 33}},
        //     // {"Left", {34, 66}},
        //     {"Left", {34, 66}},
        //     // {"Left_Rear", {67, 99}},

        //     // {"Right_Rear", {100, 133}},
        //     {"Right", {134, 165}},
        //     // {"Right", {134, 165}},
        //     {"Front_Right", {166, 187}},
        //     {"Front_Front_Right", {188, 199}},
        // };

        sectors = {
            {"Front_Front_Left",  {0, 26}},
            {"Front_Left",        {27, 76}},
            {"Left",              {77, 150}},
            {"Left_Rear",         {151, 225}},

            {"Right_Rear",        {226, 302}},
            {"Right",             {303, 374}},
            {"Front_Right",      {375, 424}},
            {"Front_Front_Right",{425, 451}},
        };
    } else {
      sectors = {
          {"Front_Front_Right", {160, 179}},
          {"Front_Front_Left", {190, 209}},
          {"Right", {60, 149}},
          {"Front_Right", {150, 179}},
          {"Front_Left", {180, 209}},
          {"Left", {210, 299}},
          {"Left_Rear", {300, 359}},
          {"Right_Rear", {0, 59}},
      };
    }

    double obstacle_threshold = is_construct_environment_ ? 0.35 : 3;

    for (const auto &sector : sectors) {

      int start_idx = sector.second.first;
      int end_idx = sector.second.second;

      float min_distance = std::numeric_limits<float>::infinity();

      if (start_idx < static_cast<int>(current_laser_scan_->ranges.size()) &&
          end_idx < static_cast<int>(current_laser_scan_->ranges.size())) {
        auto start_it = current_laser_scan_->ranges.begin() + start_idx;

        auto end_it = current_laser_scan_->ranges.begin() + end_idx + 1;

        min_distance = *std::min_element(start_it, end_it);
      }

      // Save actual distance
      result.distances[sector.first] = min_distance;

      // Save detection
      result.detections[sector.first] = min_distance < obstacle_threshold;
    }

    // for (size_t i = 0; i < current_laser_scan_->ranges.size(); ++i) {
    //   RCLCPP_INFO(this->get_logger(), "current_laser_scan_ :: %zu: %.3f m", i,
    //               current_laser_scan_->ranges[i]);
    // }

    // for (const auto &[sector, distance] : result.distances) {
    //   RCLCPP_INFO(this->get_logger(),
    //               "Result :: %s: distance=%.3f m, detected=%s", sector.c_str(),
    //               distance, result.detections[sector] ? "true" : "false");
    // }

    if (result.detections["Front_Front_Left"] ||
            result.detections["Front_Front_Right"]) {
        for (size_t i = 0; i < current_laser_scan_->ranges.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "current_laser_scan_ :: %zu: %.3f m", i,
                    current_laser_scan_->ranges[i]);
        }

        for (const auto &[sector, distance] : result.distances) {
        RCLCPP_INFO(this->get_logger(),
                    "Result :: %s: distance=%.3f m, detected=%s", sector.c_str(),
                    distance, result.detections[sector] ? "true" : "false");
        }
    }

    return result;
  }

private:
  bool is_construct_environment_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscriber_;
  sensor_msgs::msg::LaserScan::SharedPtr current_laser_scan_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto patrol_node = std::make_shared<Patrol>();

  // Overkill
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    2);
  executor.add_node(patrol_node);

  try {
    executor.spin();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(patrol_node->get_logger(), "Exception: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}