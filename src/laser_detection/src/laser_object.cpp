#include "ros/ros.h"
#include "laser_detection/laser_object.h"
#include "sensor_msgs/PointCloud2.h"
#include "laser_geometry/laser_geometry.h"
#include "tf/transform_listener.h"
#include "pcl_ros/point_cloud.h"
#include "math.h"
#include <pcl/point_types.h>

bool IsFree(pcl::PointCloud<pcl::PointXYZ> pc, double theta, double distance){
  bool is_free = true;
  for(int i=0; i<pc.points.size(); i++){
    float rotated_x = pc.points[i].x * cos(theta) - pc.points[i].y * sin(theta);
    float rotated_y = pc.points[i].x * sin(theta) + pc.points[i].y * cos(theta);
    if((rotated_y > -0.25) && (rotated_y < 0.25) ){
      if((rotated_x < 0)&&(rotated_x>-1*distance)){
        is_free = false;
        return is_free;
      }
    }
  }
  return is_free;
}

void LaserObject::LaserCallback(const sensor_msgs::LaserScanConstPtr &msg){
  laser_geometry::LaserProjection projector_;
  tf::StampedTransform transform_laser;
  try{
    listener_.lookupTransform("/base_link", "/laser", ros::Time(0), transform_laser);
  }catch (tf::TransformException ex){
    return;
  }
  sensor_msgs::PointCloud2 cloud;
  projector_.transformLaserScanToPointCloud("/laser",*msg,cloud,listener_);
  pcl::PointCloud<pcl::PointXYZ> cloud_out;
  pcl::fromROSMsg(cloud, cloud_out);
  bool back_is_free = IsFree(cloud_out, 0.0, 0.8);
  std::cout<<"back is free ? "<<back_is_free<<std::endl;
  bool left_is_free = IsFree(cloud_out, -3.14/2.0, 0.8);
  std::cout<<"left is free ?"<<left_is_free<<std::endl;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_object");
  LaserObject laser_;
//  ros::NodeHandle nh;
//  tf::TransformListener listener_;
//  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
//  ros::Subscriber laser_sub = nh.subscribe("laser", 10, LaserCallback);

  ros::spin();

  return 0;
}
