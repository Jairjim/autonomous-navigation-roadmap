#include <memory>

#include "rclcpp/rclcpp.hpp"

class RobotStateNode : public rclcpp::Node
{

public:
    RobotStateNode()
        : Node("robot_state_node")
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Robot State Node Started");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotStateNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}