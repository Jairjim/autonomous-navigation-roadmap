#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class RobotStateNode : public rclcpp::Node
{

public:
    RobotStateNode()
        : Node("robot_state_node"),
          x_(0.0),
          y_(0.0),
          theta_(0.0)
    {

        publisher_ =
            this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/robot_position",
                10);

        timer_ =
            this->create_wall_timer(
                500ms,
                std::bind(
                    &RobotStateNode::timerCallback,
                    this));

        RCLCPP_INFO(
            this->get_logger(),
            "Robot State Node Started");
    }

private:
    void timerCallback()
    {
        std_msgs::msg::Float64MultiArray message;

        message.data = {
            x_,
            y_,
            theta_};

        publisher_->publish(message);

        RCLCPP_INFO(
            this->get_logger(),
            "Published position: x=%.2f y=%.2f theta=%.2f",
            x_,
            y_,
            theta_);

        x_ += 0.1;
    }

    double x_;
    double y_;
    double theta_;

    rclcpp::Publisher<
        std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<RobotStateNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}