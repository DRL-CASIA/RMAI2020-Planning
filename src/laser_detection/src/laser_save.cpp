#include "laser_detection/laser_save.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
void LaserSave::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in){
  laser_geometry::LaserProjection projector_;
  tf::StampedTransform transform_laser;
  std::printf("what is this\n");
//  if(!listener_.waitForTransform(scan_in->header.frame_id,"/laser",
//                                 ros::Time::now(),
//                                 ros::Duration(0.2))){
//    std::printf("maybe time is wrong.\n");
//    return;
//  }
  try{
    listener_.lookupTransform("/odom", "/laser", ros::Time(0), transform_laser);
  }catch (tf::TransformException ex){
    return;
  }
  std::printf("it is ok\n");
  sensor_msgs::PointCloud cloud;
  projector_.transformLaserScanToPointCloud("/laser",*scan_in,cloud,listener_);
  PointCloud cloud_out;
  sensor_msgs::PointCloud2 pc2;
  sensor_msgs::convertPointCloudToPointCloud2(cloud, pc2);
  pcl::fromROSMsg(pc2, cloud_out);
  pcl::PointCloud<pcl::PointXYZ>::Ptr odom_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
  *laser_cloud = cloud_out;

  double roll, pitch, yaw;
  transform_laser.getBasis().getEulerYPR(yaw, pitch, roll);
  double odom_x, odom_y;
  odom_x = transform_laser.getOrigin().getX();
  odom_y = transform_laser.getOrigin().getY();
  Eigen::Matrix4f transform_ = Eigen::Matrix4f::Identity();
  transform_(0, 0) = cos(yaw);
  transform_(0, 1) = -sin(yaw);
  transform_(1, 0) = sin(yaw);
  transform_(1, 1) = cos(yaw);
  transform_(0, 3) = odom_x;
  transform_(1, 3) = odom_y;
  pcl::transformPointCloud(*laser_cloud, *odom_cloud, transform_);
  char laser_f[100];
  std::sprintf(laser_f, "/home/drl/bagfiles/pointclouds/laser_%d.pcd", laser_count);
  pcl::io::savePCDFileASCII (laser_f, *odom_cloud);
  laser_count ++;
}
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_save");
  LaserSave laser_save;

  ros::spin();

  return 0;
}
