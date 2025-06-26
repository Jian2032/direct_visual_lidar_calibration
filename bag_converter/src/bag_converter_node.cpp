#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <livox_ros_driver2/CustomMsg.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <boost/filesystem.hpp>

#include <string>
#include <sstream>

// 转换Livox CustomMsg到PCL点云
void convertLivoxCustomMsgToPCL(const livox_ros_driver2::CustomMsg::ConstPtr& msg, pcl::PointCloud<pcl::PointXYZI>& cloud) {
    cloud.clear();
    cloud.width = msg->points.size();
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width);

    for (size_t i = 0; i < msg->points.size(); ++i) {
        const auto& pt = msg->points[i];
        pcl::PointXYZI pcl_pt;
        pcl_pt.x = pt.x;
        pcl_pt.y = pt.y;
        pcl_pt.z = pt.z;
        pcl_pt.intensity = static_cast<float>(pt.reflectivity);
        cloud.points[i] = pcl_pt;
    }
}

// 保存第一帧图像
void save_image(const sensor_msgs::Image::ConstPtr& img_msg, const std::string& output_dir) {
    try {
        cv::Mat img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
        std::string filename = output_dir + "/image_0.png";
        cv::imwrite(filename, img);
        ROS_INFO("Saved first image: %s", filename.c_str());
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bag_converter");
    if (argc < 5) {
        ROS_ERROR("Usage: rosrun bag_converter bag_converter_node <bag_file> <image_topic> <pointcloud_topic> <output_dir>");
        return 1;
    }

    std::string bag_path = argv[1];
    std::string image_topic = argv[2];
    std::string pointcloud_topic = argv[3];
    std::string output_dir = argv[4];

    boost::filesystem::create_directories(output_dir);

    rosbag::Bag bag;
    try {
        bag.open(bag_path, rosbag::bagmode::Read);
    } catch (rosbag::BagIOException& e) {
        ROS_ERROR("Failed to open bag file: %s", e.what());
        return 1;
    }

    // 只遍历指定话题，避免无关消息干扰
    std::vector<std::string> topics;
    topics.push_back(image_topic);
    topics.push_back(pointcloud_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    bool saved_first_image = false;
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    int cloud_count = 0;

    for (const rosbag::MessageInstance& m : view) {
        ROS_INFO("Reading topic: %s", m.getTopic().c_str());

        // 处理图像
        if (!saved_first_image && (m.getTopic() == image_topic || ("/" + m.getTopic() == image_topic))) {
            auto img_msg = m.instantiate<sensor_msgs::Image>();
            if (img_msg) {
                ROS_INFO("Got first image message");
                save_image(img_msg, output_dir);
                saved_first_image = true;
            } else {
                ROS_WARN("Failed to instantiate sensor_msgs::Image");
            }
        }

        // 处理Livox点云
        if (m.getTopic() == pointcloud_topic || ("/" + m.getTopic() == pointcloud_topic)) {
            auto pc_msg = m.instantiate<livox_ros_driver2::CustomMsg>();
            if (pc_msg) {
                ROS_INFO("Got Livox CustomMsg with %lu points", pc_msg->points.size());
                pcl::PointCloud<pcl::PointXYZI> tmp_cloud;
                convertLivoxCustomMsgToPCL(pc_msg, tmp_cloud);
                if (!tmp_cloud.empty()) {
                    *merged_cloud += tmp_cloud;
                    ++cloud_count;
                } else {
                    ROS_WARN("Converted PCL cloud is empty");
                }
            } else {
                ROS_WARN("Failed to instantiate livox_ros_driver2::CustomMsg");
            }
        }
    }

    bag.close();

    if (merged_cloud->empty()) {
        ROS_WARN("No valid Livox point cloud data found. No output saved.");
        return 0;
    }

    std::string pcd_path = output_dir + "/merged_cloud.pcd";
    std::string ply_path = output_dir + "/merged_cloud.ply";

    pcl::io::savePCDFileBinary(pcd_path, *merged_cloud);
    pcl::io::savePLYFileBinary(ply_path, *merged_cloud);

    ROS_INFO("Merged %d Livox pointcloud frames saved to:\n - %s\n - %s", cloud_count, pcd_path.c_str(), ply_path.c_str());

    return 0;
}
