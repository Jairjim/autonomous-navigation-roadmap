#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class PositionMonitorNode : public rclcpp::Node
{
public:
    PositionMonitorNode()
        : Node("position_monitor_node")
    {
        subscription_ =
            this->create_subscription<
                std_msgs::msg::Float64MultiArray>(
                "/robot_position",
                10,
                std::bind(
                    &PositionMonitorNode::positionCallback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "Position Monitor Node started.");
    }

private:
    void positionCallback(
        const std_msgs::msg::Float64MultiArray::SharedPtr message)
    {
        if (message->data.size() < 3)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Received invalid robot position message.");

            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Received position: x=%.2f y=%.2f theta=%.2f",
            message->data[0],
            message->data[1],
            message->data[2]);
    }

    rclcpp::Subscription<
        std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<PositionMonitorNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}