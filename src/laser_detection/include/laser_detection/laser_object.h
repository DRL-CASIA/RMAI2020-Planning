#ifndef LASER_OBJECT_H
#define LASER_OBJECT_H
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"
class LaserObject
{
public:
  LaserObject() {
    laser_sub = nh.subscribe("scan", 10, &LaserObject::LaserCallback, this);
  }
  void LaserCallback(const sensor_msgs::LaserScanConstPtr &msg);
private:
  ros::NodeHandle nh;
  tf::TransformListener listener_;
  ros::Subscriber laser_sub;

};
#endif // LASER_OBJECT_H
