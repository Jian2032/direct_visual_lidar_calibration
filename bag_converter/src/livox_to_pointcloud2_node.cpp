#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>  // 请根据你的消息路径修改
#include <sensor_msgs/point_cloud2_iterator.h>

class LivoxToPointCloud2Converter
{
public:
  LivoxToPointCloud2Converter()
  {
    ros::NodeHandle nh;
    sub_ = nh.subscribe("/livox/lidar", 10, &LivoxToPointCloud2Converter::callback, this);
    pub_ = nh.advertise<sensor_msgs::PointCloud2>("/livox/pointcloud2", 10);
  }

  void callback(const livox_ros_driver2::CustomMsg::ConstPtr& msg)
  {
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header = msg->header;
    cloud_msg.height = 1;
    cloud_msg.width = msg->point_num;
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = false;

    // 自定义字段定义：x,y,z(float32) + intensity(uint8)
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::PointField::UINT8);
    modifier.resize(msg->point_num);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity(cloud_msg, "intensity");

    for (size_t i = 0; i < msg->points.size(); ++i) {
      const auto& pt = msg->points[i];
      *iter_x = pt.x;
      *iter_y = pt.y;
      *iter_z = pt.z;
      *iter_intensity = pt.reflectivity;  // 反射强度，0~255

      ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
    }

    pub_.publish(cloud_msg);
  }

private:
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "livox_to_pointcloud2_node");
  LivoxToPointCloud2Converter converter;
  ros::spin();
  return 0;
}
