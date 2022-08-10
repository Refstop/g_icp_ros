#include "g_icp_ros/g_icp_ros.h"

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

g_icp_ros::g_icp_ros(): Node("g_icp_ros"), src_ready_(false), tgt_ready_(false), src_(new pcl::PointCloud<pcl::PointXYZ>), tgt_(new pcl::PointCloud<pcl::PointXYZ>) {
    align_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/PointCloud2_align", 1);
    src_pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/PointCloud2_src", 1, std::bind(&g_icp_ros::srcCallback_, this, std::placeholders::_1));
    tgt_pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/PointCloud2_tgt", 1, std::bind(&g_icp_ros::tgtCallback_, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(100ms, std::bind(&g_icp_ros::run_, this));
}

void g_icp_ros::srcCallback_(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // convert pc2 msg to pcl::PointCloud<pcl::PointXYZ>
    pcl::fromROSMsg(*msg, *src_);
    src_ready_ = true;
}

void g_icp_ros::tgtCallback_(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::fromROSMsg(*msg, *tgt_);
    tgt_ready_ = true;
}

void g_icp_ros::run_() {
    // test by example bin file
    // test_by_bin_file_(src_, tgt_);
    if(src_ready_ && tgt_ready_) {
        cout << "Execute G-ICP" << endl;
        /**
         * Main
         */
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setMaxCorrespondenceDistance(1.0);
        gicp.setTransformationEpsilon(0.003);
        gicp.setMaximumIterations(1000);
        pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);

        gicp.setInputSource(src_);
        gicp.setInputTarget(tgt_);
        gicp.align(*align);

        /*******************************************/
        // Set outputs
        Eigen::Matrix4f src2tgt = gicp.getFinalTransformation();
        double score = gicp.getFitnessScore();
        bool is_converged = gicp.hasConverged();

        cout<<src2tgt<<endl;
        cout<<score<<endl;

        pcl::transformPointCloud(*tgt_, *align, src2tgt.inverse());

        auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*align, *pc2_msg);
        pc2_msg->header.frame_id = "map";
        pc2_msg->header.stamp = now();

        align_pc_pub_->publish(*pc2_msg);

        src_ready_ = tgt_ready_ = false;
    }
    
}

void g_icp_ros::test_by_bin_file_(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt) {
    std::string bin_dir = "/run/user/1000/gvfs/afp-volume:host=synology-NAS.local,user=sparo,volume=SLAM_Dataset/KITTI_Dataset/Obometry/data_odometry_velodyne/dataset/sequences/00/velodyne/";
    *src = *load_bin(bin_dir + "000000.bin");
    *tgt = *load_bin(bin_dir + "000001.bin");
    src_ready_ = tgt_ready_ = true;
}