#ifndef LASER_TRACKING_H
#define LASER_TRACKING_H
#include "ros/ros.h"
#include "roborts_msgs/ArmorPos.h"
#include "roborts_msgs/ArmorsPos.h"
#include "nav_msgs/Path.h"
#include <list>
#include "tf/transform_listener.h"
//#include "roborts_msgs/EnemiesPos.h"

struct TrackingPoint
{
  double x;
  double y;
  double vx;
  double vy;
};
struct Point
{
  double x;
  double y;
};
double findNearDistanceToPointVec(Point p, std::vector<Point> p_vec){
  if(p_vec.size() == 0){
    return 100.0;
  }else{
    double near_distance = sqrt((p.x - p_vec[0].x) * (p.x - p_vec[0].x) +
                                (p.y - p_vec[0].y) * (p.y - p_vec[0].y));
    for(int i=1; i<p_vec.size(); i++){
      double current_distance = sqrt((p.x - p_vec[i].x) * (p.x - p_vec[i].x) +
                                      (p.y - p_vec[i].y) * (p.y - p_vec[i].y));
      if(current_distance < near_distance)
        near_distance = current_distance;
    }
    return near_distance;
  }
}
double getDistance(Point p, TrackingPoint tp){
  return sqrt((p.x-tp.x)*(p.x-tp.x) + (p.y-tp.y)*(p.y-tp.y));
}

void printTrackingVec(std::list<TrackingPoint> p_vec, std::list<int> p_c){
  std::cout<<"Point vec size: "<<p_vec.size()<<std::endl;
  std::list<int>::iterator it_i = p_c.begin();
  for(std::list<TrackingPoint>::iterator it=p_vec.begin(); it!=p_vec.end(); it++, it_i++){
    std::cout<<"x: "<<it->x<<" y: "<<it->y<<" leak detection: "<<*it_i<<std::endl;
  }
}

void printTrackingPath(std::list<std::list<TrackingPoint> > path){
  std::cout<<"==============================="<<std::endl;
  std::cout<<"Number of paths: "<<path.size()<<std::endl;
  int c = 0;
  for(std::list<std::list<TrackingPoint> >::iterator it=path.begin();
      it!=path.end(); it++){
    std::cout<<"Path "<<c<<" size: "<<it->size()<<std::endl;
    c++;
  }
  std::cout<<"==============================="<<std::endl;
}

void printDiedCars(std::vector<Point> points){
  std::cout<<"==============================="<<std::endl;
  for(int i=0; i<points.size(); i++){
    std::cout<<"x: "<<points[i].x<<" y: "<<points[i].y<<std::endl;
  }
  std::cout<<"==============================="<<std::endl;
}
class LaserTracking
{
public:
  LaserTracking() {
    tracking_enemies_.clear();
    leak_detected_count_.clear();
    delete_count_thresh_ = 10;
    distance_thresh_ = 0.6;
    camer_laser_thresh_ = 0.6;
    enemies_for_decision_.num_armor = 0;
    camera_history_size_ = 0;
    std::vector<float> invalid_armor;
    invalid_armor.push_back(-10);
    invalid_armor.push_back(-10);
    enemies_for_decision_.armor_0 = invalid_armor;
    enemies_for_decision_.armor_1 = invalid_armor;
    is_shooter_camera_valid_ = false;
    robo_x_ = 0.0; robo_y_ = 0.0; robo_yaw_ = 0.0; updated_robo_state_ = false;
    enemies_sub_ = nh_.subscribe("enemies", 1, &LaserTracking::ReceiveEnemies, this);
    enemies_pub_ = nh_.advertise<roborts_msgs::ArmorsPos>("laser_tracking_enemies", 10);
    path_0_pub_ = nh_.advertise<nav_msgs::Path>("enemy_0_path", 10);
    path_1_pub_ = nh_.advertise<nav_msgs::Path>("enemy_1_path", 10);

    camera_armors_sub_ = nh_.subscribe("armors_camera", 1, &LaserTracking::UpdateCameraBuffer, this);
    shooter_camera_sub_ = nh_.subscribe("armor_pos", 1, &LaserTracking::UpdateShooterImage, this);

  }
  void ReceiveEnemies(const roborts_msgs::ArmorsPosConstPtr &msg);
  void UpdateTracking(std::vector<Point> points);
  void UpdatePoint(Point point, int id);
  void UpdateAllLeakDetected();
  void AddTrackingPoint(Point point);
  int GetNearPointId(Point p, std::list<TrackingPoint> p_vec);
  void PublishTrackingEnemies(std::list<TrackingPoint> p_vec);

  void UpdateTrackingPath(Point point, int id);
  void AddTrackingPath(Point point);
  void UpdateTrackingWindow();
  void PublishTrackingPath(std::list<std::list<TrackingPoint> > path);
  void PublishForDecision(std::vector<TrackingPoint> points);

  // robo pose
  void UpdateRoboState();
  void PublishActivePoint(std::vector<Point> points);

  // camera
  void UpdateCameraBuffer(const roborts_msgs::ArmorsPosConstPtr &msg);
  bool isLaserInCameraView(TrackingPoint p);
  void FusionLaserCamera(std::vector<TrackingPoint> laser_enemies, std::vector<Point> camera_enemies);
  void UpdateDiedCar(Point p);
  void RemoveDiedCar(Point p);
  std::vector<Point> HistoryVecToBuffer(std::list<std::vector<Point> > history_list);

  void UpdateShooterImage(const roborts_msgs::ArmorPosConstPtr &msg);
private:
  ros::NodeHandle nh_;
  ros::Subscriber enemies_sub_, camera_armors_sub_,shooter_camera_sub_;
  ros::Publisher enemies_pub_, path_0_pub_, path_1_pub_;
  std::vector<Point> received_enemies_;
  std::list<TrackingPoint> tracking_enemies_;
  std::vector<TrackingPoint> publish_enemies_;
  std::list<std::list<TrackingPoint> > tracking_path_;
  std::list<int> leak_detected_count_;
  int delete_count_thresh_;
  double distance_thresh_, camer_laser_thresh_;

  // robo state
  tf::TransformListener tf_listener_;
  double robo_x_, robo_y_, robo_yaw_;
  bool updated_robo_state_;

  roborts_msgs::ArmorsPos enemies_for_decision_;

  // camera
  std::vector<Point> camera_armor_buffer_;
  std::vector<Point> died_cars;
  bool is_shooter_camera_valid_;
  Point shooter_camera_point_;
  int camera_history_size_;
  std::list<std::vector<Point> > camera_history_buffer_;
};
#endif // LASER_TRACKING_H
