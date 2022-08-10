#ifndef G_ICP_ROS
#define G_ICP_ROS

#include <iostream>
#include <chrono>
#include <memory>
// #include <cstdint>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/point_cloud2.hpp>


using namespace std;
using namespace std::chrono_literals;

class g_icp_ros: public rclcpp::Node {
    public:
        g_icp_ros();
        ~g_icp_ros() {};

    private:
        void run_();
        void srcCallback_(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void tgtCallback_(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void test_by_bin_file_(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt);
        pcl::PointCloud<pcl::PointXYZ>::Ptr src_, tgt_;
        bool src_ready_, tgt_ready_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr align_pc_pub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr src_pc_sub_, tgt_pc_sub_;
};

#endif