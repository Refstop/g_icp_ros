#include "pcl_publisher/pcl_publisher.h"

pcl::PointCloud<pcl::PointXYZ>::ConstPtr load_bin(const string &filename) {
    FILE*file = fopen(filename.c_str(), "rb");
    if (!file) {
        std::cerr << "Error: failed to load " << filename << std::endl;
        return nullptr;
    }
    std::vector<float> buffer(1000000);
    size_t num_points = fread(reinterpret_cast<char*>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
    fclose(file);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->resize(num_points);

    for (int i = 0; i < num_points; i++) {
        auto &pt = cloud->at(i);
        pt.x = buffer[i * 4];
        pt.y = buffer[i * 4 + 1];
        pt.z = buffer[i * 4 + 2];
        // pt.intensity = buffer[i * 4 + 3];
    }

    return cloud;
}

pcl_publisher::pcl_publisher(): Node("pcl_publisher"), src_ready_(false), tgt_ready_(false), src_(new pcl::PointCloud<pcl::PointXYZ>), tgt_(new pcl::PointCloud<pcl::PointXYZ>), count_(0) {
    bin_dir_ = "/home/sparo/data/odom_velodyne/00/velodyne/";
    pc_src_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/PointCloud2_src", 1);
    pc_tgt_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/PointCloud2_tgt", 1);
    timer_ = this->create_wall_timer(100ms, std::bind(&pcl_publisher::run_, this));
}

void pcl_publisher::run_() {
    test_by_bin_file_(src_, tgt_);
    if(src_ready_ && tgt_ready_) {
        auto pc_src_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*src_, *pc_src_msg);
        pc_src_msg->header.frame_id = "map";
        pc_src_msg->header.stamp = now();

        pc_src_pub_->publish(*pc_src_msg);

        auto pc_tgt_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*tgt_, *pc_tgt_msg);
        pc_tgt_msg->header.frame_id = "map";
        pc_tgt_msg->header.stamp = now();

        pc_tgt_pub_->publish(*pc_tgt_msg);
        src_ready_ = tgt_ready_ = false;
    }
    
}

void pcl_publisher::test_by_bin_file_(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt) {
    if(count_ >= 4540) return;
    std::string num_str = (boost::format("%06d.bin") % count_).str();

    *src = *load_bin(bin_dir_ + num_str);
    num_str = (boost::format("%06d.bin") % ++count_).str();
    *tgt = *load_bin(bin_dir_ + num_str);
    src_ready_ = tgt_ready_ = true;
}