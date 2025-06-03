#include "fleet_controller/controller.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FleetController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
