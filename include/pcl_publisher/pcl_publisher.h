#ifndef PCL_PUBLISHER
#define PCL_PUBLISHER

#include <iostream>
#include <chrono>
#include <memory>
#include <boost/format.hpp>

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

class pcl_publisher: public rclcpp::Node {
    public:
        pcl_publisher();
        ~pcl_publisher() {};
    
    private:
        void run_();
        void test_by_bin_file_(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt);
        pcl::PointCloud<pcl::PointXYZ>::Ptr src_, tgt_;
        bool src_ready_, tgt_ready_;
        size_t count_;
        std::string bin_dir_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_src_pub_, pc_tgt_pub_;
};

#endif