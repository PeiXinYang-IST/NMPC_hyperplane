#include "rclcpp/rclcpp.hpp"
#include "nmpc_tracker_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NmpcTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}