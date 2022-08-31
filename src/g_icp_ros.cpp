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

g_icp_ros::g_icp_ros():
    Node("g_icp_ros"),
    src_ready_(false),
    tgt_ready_(false),
    src_(new pcl::PointCloud<pcl::PointXYZ>), 
    tgt_(new pcl::PointCloud<pcl::PointXYZ>),
    laser_tf_(MatrixXf::Identity(4, 4)) {
    num1 = num2 = 0;
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    align_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/PointCloud2_align", 1);
    src_pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/PointCloud2_src", 1, std::bind(&g_icp_ros::srcCallback_, this, std::placeholders::_1));
    tgt_pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/PointCloud2_tgt", 1, std::bind(&g_icp_ros::tgtCallback_, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, std::bind(&g_icp_ros::scanCallback_, this, std::placeholders::_1));
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

void g_icp_ros::scanCallback_(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::vector<float> ranges = msg->ranges;
    pcl::PointCloud<pcl::PointXYZ> src, tgt;
    if(!init_) {
        for(int i = 0; i < ranges.size(); i++) {
            if(ranges[i] != 0) {
                src.push_back(pcl::PointXYZ(ranges[i]*cos(-msg->angle_min - i*msg->angle_increment), ranges[i]*sin(-msg->angle_min - i*msg->angle_increment), 0));
            }
        }
        *src_ = src;
        init_ = true;
    }
    else {
        ++num1 %= 300;
        for(int i = 0; i < ranges.size(); i++) {
            if(ranges[i] != 0) {
                tgt.push_back(pcl::PointXYZ(ranges[i]*cos(-msg->angle_min - i*msg->angle_increment), ranges[i]*sin(-msg->angle_min - i*msg->angle_increment), 0));
            }
        }
        *tgt_ = tgt;
        scan_ready_ = true;
    }

    
}

void g_icp_ros::run_() {
    // test by example bin file
    // test_by_bin_file_(src_, tgt_);
    if(src_ready_ && tgt_ready_) {
        cout << "Execute G-ICP(PointCloud2)" << endl;
        /*
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
    else if(scan_ready_) {
        cout << "Execute G-ICP(LaserScan)" << endl;
        /*
         * Main
         */
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setMaxCorrespondenceDistance(1.0);
        gicp.setTransformationEpsilon(0.003);
        gicp.setMaximumIterations(1000);
        pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);

        cout << "num1: " << num1 << "num2: " << num2 << endl;
        gicp.setInputSource(src_);
        gicp.setInputTarget(tgt_);
        gicp.align(*align);

        /*******************************************/
        // Set outputs
        Eigen::Matrix4f src2tgt = gicp.getFinalTransformation();
        double score = gicp.getFitnessScore();
        bool is_converged = gicp.hasConverged();
        cout << src2tgt << endl;
        // cout<<src2tgt<<endl;
        // cout<<score<<endl;
        laser_tf_ = src2tgt * laser_tf_;

        double x = laser_tf_(0,3), y = laser_tf_(1,3), theta = atan2(laser_tf_(0,1), laser_tf_(0,0));
        // cout << "x: " << x << " y: " << y << " theta: " << theta << endl;
        // cout << laser_tf_ << endl;

        publish_odom_tf_(x, y, theta, "odom", "laser");
        *src_ = *tgt_;
        num2 = num1;
        scan_ready_ = false;
    }
}

void g_icp_ros::publish_odom_tf_(double x, double y, double theta, string parent_coord, string child_coord) {
    rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = now;
    t.header.frame_id = parent_coord;
    t.child_frame_id = child_coord;

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
}

void g_icp_ros::test_by_bin_file_(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt) {
    std::string bin_dir = "/run/user/1000/gvfs/afp-volume:host=synology-NAS.local,user=sparo,volume=SLAM_Dataset/KITTI_Dataset/Obometry/data_odometry_velodyne/dataset/sequences/00/velodyne/";
    *src = *load_bin(bin_dir + "000000.bin");
    *tgt = *load_bin(bin_dir + "000001.bin");
    src_ready_ = tgt_ready_ = true;
}