#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "robot_patrol/srv/get_direction.hpp"

class TestService : public rclcpp::Node {
    public:
        TestService() : Node("test_service") {
            is_construct_environment_ = true;

            std::string laser_subscriber_name = is_construct_environment_ ? "/fastbot_1/scan" : "/vehicle_green/scan";
            laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                laser_subscriber_name, 10,
                std::bind(&TestService::laser_callback, this, std::placeholders::_1));

            std::string name_service = "/direction_service";
            client_ = this->create_client<robot_patrol::srv::GetDirection>(name_service);

            while (!client_->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                    return;
                }

                RCLCPP_INFO(this->get_logger(), "Service %s not available, waiting again...", name_service.c_str());
            }

            control_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&TestService::send_request, this));

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
            [this](rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future) {
                    auto response = future.get();

                    RCLCPP_INFO(
                        this->get_logger(),
                        "Direction decision: %s",
                        response->direction.c_str()
                    );
                }
            );
        }

    private:
        bool is_construct_environment_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
        sensor_msgs::msg::LaserScan::SharedPtr current_laser_scan_;
        rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto client = std::make_shared<TestService>();

    rclcpp::spin(client);

    rclcpp::shutdown();

    return 0;
}