#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pcl_ros/point_cloud.h"
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/segment_differences.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const sensor_msgs::PointCloudConstPtr &msg){
  PointCloud cloud_out;
  sensor_msgs::PointCloud2 pc2;
  sensor_msgs::convertPointCloudToPointCloud2(*msg, pc2);
  pcl::fromROSMsg(pc2, cloud_out);
  pcl::io::savePCDFileASCII ("/home/drl/ros_codes/catkin_ws/src/laser_detection/map_cloud/map_pointcloud_icra.pcd", cloud_out);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source ( new pcl::PointCloud<pcl::PointXYZ> () );
  *cloud_source = cloud_out;
  pcl::SegmentDifferences<pcl::PointXYZ> seg;
  seg.setInputCloud(cloud_source);
  std::printf ("received pc!\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_save");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud>("PointCloudMap", 1, callback);

  ros::spin();

  return 0;
}
