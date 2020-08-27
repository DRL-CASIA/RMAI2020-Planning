#include "laser_detection/cost_map.h"
void CostMapCV::Map2World(unsigned int mx, unsigned int my, float &wx, float &wy){
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}
bool CostMapCV::World2Map(float wx, float wy, unsigned int &mx, unsigned int &my){
  if (wx < origin_x_ || wy < origin_y_) {
      return false;
    }
    mx = (int) ((wx - origin_x_) / resolution_);
    my = (int) ((wy - origin_y_) / resolution_);
    if (mx < size_x_ && my < size_y_) {
      return true;
    }
    return false;
}
void CostMapCV::GenerateSemiCircle(float r){
  float begin_radius = 3.14159 / 2.0;
  float end_radius = 3.14159*3.0 / 2.0;
  float step = 0.01;
  int num_points = int((end_radius - begin_radius) / step) + 1;
  semi_circle.clear();
  for(int i=0; i<num_points; i++){
    float theta = begin_radius + step * float(i);
    cv::Point2f p;
    p.x = r * cos(theta);
    p.y = r * sin(theta);
    semi_circle.push_back(p);
  }
}
void CostMapCV::CostmapCallback(const nav_msgs::OccupancyGridConstPtr &msg){
  std::cout<<"height: "<<msg->info.height<<" width: "<<msg->info.width<<std::endl;
  cv::Mat grid_map = cv::Mat(msg->info.height, msg->info.width,CV_8S, const_cast<int8_t*>(&msg->data[0]), (size_t)msg->info.width);
  cost_map.clear();
  for(int i=0; i<msg->data.size(); i++){
    cost_map.push_back(msg->data[i]);
  }
  cv::Point2f back_point;
  if(FindFeasiblePoint(1.5, back_point)){
    unsigned int mx, my;
    if(World2Map(back_point.x, back_point.y, mx, my)){
      std::cout<<"pixel: "<<mx<<" ,"<<my<<std::endl;
      cv::Point p_show(mx, my);
      cv::circle(grid_map, p_show, 3, cv::Scalar(0,255,0), -1, 8);
    }
  }
  cv::imshow("grid_map", grid_map);
  cv::waitKey(1);
//  cost_map = msg->data;
}
bool CostMapCV::FindFeasiblePoint(float r, cv::Point2f &point){
  // lookup baselink
  tf::StampedTransform transform_base;
  try{
    listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform_base);
  }catch (tf::TransformException ex){
    return false;
  }
  float base_x = transform_base.getOrigin().x();
  float base_y = transform_base.getOrigin().y();
  std::cout<<"current position: "<<base_x<<" ,"<<base_y<<std::endl;
  double roll, pitch, yaw;
  transform_base.getBasis().getEulerYPR(yaw, pitch, roll);
  yaw = -yaw;
  // generate circle points
  GenerateSemiCircle(r);
  // map points to pixels and find feasible points
  std::vector<cv::Point2f> feasible_points;
  for(int i=0; i<semi_circle.size(); i++){
    float p_x = base_x + semi_circle[i].x * cos(yaw) + semi_circle[i].y * sin(yaw);
    float p_y = base_y - semi_circle[i].x * sin(yaw) + semi_circle[i].y * cos(yaw);
    unsigned int mx, my;
    cv::Point2f p;
    p.x = p_x;
    p.y = p_y;
    if(World2Map(p_x, p_y, mx, my)){
      int cost = cost_map[GetIndex(mx, my)];
      if(cost < 10)
        feasible_points.push_back(p);
    }
  }
  if (feasible_points.size() == 0){
    std::printf("No feasible point! \n");
    return false;
  }else{ // select best points
    float min_angle = 3.14;
    cv::Point2f best_point(base_x, base_y);
    for(int i=0; i<feasible_points.size(); i++){
      float angle = atan((feasible_points[i].y - base_y) / (feasible_points[i].x - base_x));
      if(fabs(angle)<fabs(min_angle))
      {
        best_point.x = feasible_points[i].x;
        best_point.y = feasible_points[i].y;
      }
    }
    if((best_point.x == base_x) && (best_point.y == base_y)){
      std::printf("Point is bad! \n");
      return false;
    }else{
      point.x = best_point.x;
      point.y = best_point.y;
      std::cout<<"feasible point: "<<point.x<<" ,"<<point.y<<std::endl;
      return true;
    }
  }

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cost_map_points");
  CostMapCV cost_map_;
  ros::spin();

  ROS_INFO("Hello world!");
}
