#include "g_icp_ros/g_icp_ros.h"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g_icp_ros>());
    rclcpp::shutdown();
    return 0;
}
