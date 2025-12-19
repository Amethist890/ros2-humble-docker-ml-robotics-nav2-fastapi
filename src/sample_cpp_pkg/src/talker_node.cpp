#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * @brief Simple talker node that publishes string messages
 */
class Talker : public rclcpp::Node {
public:
    Talker() : Node("talker"), count_(0) {
        // Create publisher on 'chatter' topic
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
        
        // Create timer for periodic publishing (500ms interval)
        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&Talker::timer_callback, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Talker node started");
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello from C++ ROS 2! Count: " + std::to_string(count_++);
        
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();
    return 0;
}

