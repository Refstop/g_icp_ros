#include "pcl_publisher/pcl_publisher.h"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pcl_publisher>());
    rclcpp::shutdown();
    return 0;
}
