#ifndef COST_MAP_H
#define COST_MAP_H
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "opencv2/opencv.hpp"
#include "math.h"
#include "tf/transform_listener.h"

class CostMapCV
{
public:
  CostMapCV() {
    origin_x_ = -5.0;
    origin_y_ = -5.0;
    resolution_ = 0.05;
    size_x_ = 448;
    size_y_ = 192;
    costmap_sub_ = nh_.subscribe("/costmap/costmap/costmap", 1, &CostMapCV::CostmapCallback, this);
  }
  void CostmapCallback(const nav_msgs::OccupancyGridConstPtr &msg);
  void Map2World(unsigned int mx, unsigned int my, float &wx, float &wy);
  bool World2Map(float wx, float wy, unsigned int &mx, unsigned int &my);
  inline unsigned int GetIndex(unsigned int mx, unsigned int my) const {
      return my * size_x_ + mx;
    }
  void GenerateSemiCircle(float r);
  bool FindFeasiblePoint(float r, cv::Point2f &point);
private:
  ros::NodeHandle nh_;
  ros::Subscriber costmap_sub_;
  tf::TransformListener listener_;
  float origin_x_, origin_y_, resolution_;
  int size_x_, size_y_;
  std::vector<cv::Point2f> semi_circle;
  std::vector<int> cost_map;
};
#endif // COST_MAP_H
