#ifndef LASER_SAVE_H
#define LASER_SAVE_H
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "laser_geometry/laser_geometry.h"
#include "tf/transform_listener.h"
#include "pcl_ros/point_cloud.h"
#include <pcl/point_types.h>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "math.h"
class LaserSave
{
public:
  LaserSave() {
    laser_count = 0;
    sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, &LaserSave::scanCallback,this);
  }
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
private:
  tf::TransformListener listener_;
  ros::NodeHandle nh;
  int laser_count;


  ros::Subscriber sub;
};
#endif // LASER_SAVE_H
