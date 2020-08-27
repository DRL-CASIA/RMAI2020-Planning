#ifndef LASER_DETECTION_H
#define LASER_DETECTION_H
#include <ros/ros.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <nav_msgs/GetMap.h>
#include "laser_geometry/laser_geometry.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "pcl_ros/point_cloud.h"
#include "sensor_msgs/PointCloud.h"
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/point_cloud_conversion.h"
#include "roborts_msgs/ArmorPos.h"
#include "roborts_msgs/ArmorsPos.h"
#include "zmq.hpp"

struct Point{
  float x;
  float y;
};
struct Pose{
  float x;
  float y;
  float vx;
  float vy;
};

float getPointDistance(Point p1, Point p2){
  return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
}

void nearstPoint(Point p_in, std::vector<Point> p_vec, Point &p_out, float &min_dist){
  min_dist = getPointDistance(p_in, p_vec[0]);
  int min_idx = 0;
  for(int i=1; i<p_vec.size(); i++){
    float current_d = getPointDistance(p_in, p_vec[i]);
    if(current_d < min_dist){
      min_dist = current_d;
      min_idx = i;
    }
  }
  p_out = p_vec[min_idx];
}

class LaserDetection
{
public:
  LaserDetection(): _context(1),
  _clientSocket(_context, ZMQ_REQ){
    _clientSocket.connect ("tcp://192.168.1.127:5555");
    static_map_topic_ = "/static_map";
    map_frame_ = "/map";
    laser_frame_ = "base_laser_link";
    laser_tracking_frame_ = "laser_tracking";
    gimbal_link_frame_ = "gimbal_link";
    self_pose_.x = 0.0;
    self_pose_.y = 0.0;
    is_initialized_tracking_ = false;
    is_received_friend_ = false;
    is_first_detection = true;
    friend_enemy_dist_thresh_ = 0.4;
    last_detected_enemies_.num_armor = 0;
    std::vector<float> invalid_pos;
    invalid_pos.push_back(-10);
    invalid_pos.push_back(-10);
    last_detected_enemies_.armor_0 = invalid_pos;
    last_detected_enemies_.armor_1 = invalid_pos;
    map_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    odom_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    LoadMap();
    std::string tf_prefix = tf::getPrefixParam(nh);
    std::string scan_frame = "scan";
   
    if (!tf_prefix.empty()){
        scan_frame = "base_scan";
        laser_frame_ = tf::resolve(tf_prefix, laser_frame_);
        laser_tracking_frame_ = tf::resolve(tf_prefix, laser_tracking_frame_);
        gimbal_link_frame_ = tf::resolve(tf_prefix, gimbal_link_frame_);
    }
    laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>(scan_frame, 1, &LaserDetection::UpdateLaser,this);  // automatically add /scale
    enemy_pub_ = nh.advertise<roborts_msgs::ArmorPos>("laser_armor", 10);
    points_pub_ = nh.advertise<roborts_msgs::ArmorsPos>("enemies", 10);
    seg_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("dynamic_obstacle", 10);
    in_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("in_map_pc", 10);
  }
  void LoadMap();
  void UpdateLaser(const sensor_msgs::LaserScanConstPtr &msg);
  void Detection(pcl::PointCloud<pcl::PointXYZ>::Ptr laser);
  void Tracking(std::vector<Point> points);
  void PublishTF(Pose pose);
  void PublishMsg(bool is_valid, Pose pose);
  void PublishSegmentPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr seg_laser);
  void PublishInMapPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr seg_laser);
  std::vector<Point> FilterFriend(std::vector<Point> points);
  void LoopupFriend();
  void PublishPoints(std::vector<Point> points);
private:
  tf::TransformListener listener_;
  tf::TransformBroadcaster enemy_tf;
  ros::NodeHandle nh;
  ros::Subscriber laser_sub_;
  ros::Publisher enemy_pub_, seg_pc_pub_, in_map_pub_, points_pub_;
  Pose estimate_pose_;
  Point self_pose_, friend_;
  bool is_initialized_tracking_, is_received_friend_, is_first_detection;
  int continue_no_points;
  double friend_enemy_dist_thresh_;
  std::vector<Point> laset_detections_;

  std::string static_map_topic_, map_frame_, laser_frame_, laser_tracking_frame_, gimbal_link_frame_;

  roborts_msgs::ArmorsPos last_detected_enemies_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud, odom_cloud;

  // zmq
  zmq::context_t _context;
  zmq::socket_t  _clientSocket;
};
#endif // LASER_DETECTION_H
