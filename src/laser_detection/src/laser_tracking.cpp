#include "laser_detection/laser_tracking.h"
void LaserTracking::UpdateShooterImage(const roborts_msgs::ArmorPosConstPtr &msg){
  if(msg->is_valid){
    is_shooter_camera_valid_ = true;
    shooter_camera_point_.x = msg->x;
    shooter_camera_point_.y = msg->y;
  }else{
    is_shooter_camera_valid_ = false;
  }
}
bool LaserTracking::isLaserInCameraView(TrackingPoint p){
  // project laser in to base_link frame
  double delta_x = p.x - robo_x_; double delta_y = p.y - robo_y_;
  double new_x =   delta_x * cos(robo_yaw_) + delta_y * sin(robo_yaw_);
  double new_y = - delta_x * sin(robo_yaw_) + delta_y * cos(robo_yaw_);
//  double distance = sqrt(delta_x * delta_x + delta_y * delta_y);
//  if(distance < 1.0){
//    ROS_WARN("car is near to robo, distance: %f", distance);
//    return false;
//  }
  double new_yaw = atan2(new_y, new_x) * 180 / 3.1415926;
  ROS_WARN("car in map: %f, %f; car in robo: %f, %f, yaw: %f", p.x, p.y, delta_x, delta_y, new_yaw);
  if((new_yaw > -15) && (new_yaw < 15))
    return true;
  if((new_yaw > 120) && (new_yaw < 150))
    return true;
  if((new_yaw > -150) && (new_yaw < -120))
    return true;
  return false;
}
void LaserTracking::UpdateRoboState(){
  tf::StampedTransform transform_pose_;
  try{
    tf_listener_.lookupTransform("map", "base_link", ros::Time(0), transform_pose_);
    robo_x_ = transform_pose_.getOrigin().x();
    robo_y_ = transform_pose_.getOrigin().y();
    double roll, pitch;
    transform_pose_.getBasis().getEulerYPR(robo_yaw_, pitch, roll);
    updated_robo_state_ = true;
  }catch(tf::TransformException ex){
    ROS_WARN("Cannot find the robo pose in map, localization node maybe die");
  }
}
void LaserTracking::UpdateDiedCar(Point p){
  if(died_cars.size() == 0){
    ROS_INFO("add died car");
    died_cars.push_back(p);
  }else{
    TrackingPoint p_t;
    p_t.x = p.x; p_t.y = p.y; p_t.vx = 0.0; p_t.vy = 0.0;
    double near_distance = getDistance(died_cars[0], p_t);
    int idx = 0;
    for(int i=1; i<died_cars.size(); i++){
      double current_distance = getDistance(died_cars[i], p_t);
      if(current_distance < near_distance){
        near_distance = current_distance;
        idx = i;
      }
    }
    if(near_distance <= distance_thresh_){
      died_cars[idx].x = (died_cars[idx].x + p.x) / 2;
      died_cars[idx].y = (died_cars[idx].y + p.y) / 2;
    }else{
      ROS_INFO("add died car");
      died_cars.push_back(p);
    }
  }
}
void LaserTracking::RemoveDiedCar(Point p){
  if(died_cars.size() == 0){
    return;
  }else{
    TrackingPoint p_t;
    p_t.x = p.x; p_t.y = p.y; p_t.vx = 0.0; p_t.vy = 0.0;
    double near_distance = getDistance(died_cars[0], p_t);
    int idx = 0;
    for(int i=1; i<died_cars.size(); i++){
      double current_distance = getDistance(died_cars[i], p_t);
      if(current_distance < near_distance){
        near_distance = current_distance;
        idx = i;
      }
    }
//    ROS_WARN("distance: %f", near_distance);
    if(near_distance <= camer_laser_thresh_){
      std::vector<Point>::iterator it = died_cars.begin()+idx;
      ROS_WARN("Remove died car use camera results: %f, %f", p.x, p.y);
      died_cars.erase(it);
    }
  }
}
void LaserTracking::UpdateCameraBuffer(const roborts_msgs::ArmorsPosConstPtr &msg){
  camera_armor_buffer_.clear();
  if(msg->num_armor == 0){
//    return;
  }else if (msg->num_armor == 1) {
    Point p;
    p.x = msg->armor_0[0];
    p.y = msg->armor_0[1];
    camera_armor_buffer_.push_back(p);
//    return;
  }else if (msg->num_armor == 2) {
    Point p;
    p.x = msg->armor_0[0];
    p.y = msg->armor_0[1];
    camera_armor_buffer_.push_back(p);
    p.x = msg->armor_1[0];
    p.y = msg->armor_1[1];
    camera_armor_buffer_.push_back(p);
//    return;
  }
  camera_history_buffer_.push_back(camera_armor_buffer_);
  if(camera_history_size_ > 10){
    camera_history_buffer_.pop_front();
  }
//  ROS_ERROR("camera history buffer size: %d", int(camera_history_buffer_.size()));
  camera_history_size_ ++;
}
void LaserTracking::FusionLaserCamera(std::vector<TrackingPoint> laser_enemies, std::vector<Point> camera_enemies){
  std::vector<Point> active_cars;
  ROS_INFO("camera find %d cars, laser find %d cars", int(camera_enemies.size()), int(laser_enemies.size()));
  if(false){
    for(int i=0; i<laser_enemies.size(); i++){
      Point p;
      p.x = laser_enemies[i].x; p.y = laser_enemies[i].y;
      active_cars.push_back(p);
    }
  }else{
    for(int i=0; i<laser_enemies.size(); i++){
      // is in camera view ?
      Point car;
      car.x = laser_enemies[i].x; car.y = laser_enemies[i].y;
      if(updated_robo_state_){
        if(isLaserInCameraView(laser_enemies[i])){
          if(camera_enemies.size()>0){
            // find near camera
            double near_distance = getDistance(camera_enemies[0], laser_enemies[i]);
            for(int i_c=0; i_c<camera_enemies.size(); i_c ++){
              double dist = getDistance(camera_enemies[i_c], laser_enemies[i]);
              if(dist < near_distance){
                near_distance = dist;
              }
            }
            if(near_distance > camer_laser_thresh_){
              // update died car
              ROS_INFO("find laser car in camera view, but not in camera, update died car vector");
              UpdateDiedCar(car);
            }else{
              // update active car
              active_cars.push_back(car);
            }
          }else{
            UpdateDiedCar(car);
          }
        }else{ // not in camera view
          ROS_INFO("laser not in camera view");
          active_cars.push_back(car);
        }
      }
    }
    //
    for(int i=0; i<camera_enemies.size(); i++){
      RemoveDiedCar(camera_enemies[i]);
      if(laser_enemies.size() > 0){ //camera detected but laser not detected
        double near_laser_distance = getDistance(camera_enemies[i], laser_enemies[0]);
        for(int j=1; j<laser_enemies.size(); j++){
          double dist = getDistance(camera_enemies[i], laser_enemies[j]);
          if(dist < near_laser_distance){
            near_laser_distance = dist;
          }
        }
        if(near_laser_distance > camer_laser_thresh_){
          active_cars.push_back(camera_enemies[i]);
        }
      }else{
        active_cars.push_back(camera_enemies[i]);
      }
    }
  }
  // remove died car use shooter camera
  if(is_shooter_camera_valid_){
    ROS_WARN("use shooter camera");
    RemoveDiedCar(shooter_camera_point_);
  }
  // update active car by died car
  std::vector<Point> update_active_cars;
  for(int i=0; i<active_cars.size(); i++){
    double distance = findNearDistanceToPointVec(active_cars[i], died_cars);
    if(distance > distance_thresh_){
      update_active_cars.push_back(active_cars[i]);
    }
  }
  std::cout<<"Died cars number: "<<died_cars.size()<<std::endl;
  printDiedCars(died_cars);
  //
//  if(is_shooter_camera_valid_){
//    double distance = findNearDistanceToPointVec(shooter_camera_point_, update_active_cars);
//    if(distance > distance_thresh_){
//      ROS_INFO("add car by shoot camera");
//      update_active_cars.push_back(shooter_camera_point_);
//    }
//  }
  std::cout<<"Active cars number: "<<update_active_cars.size()<<std::endl;
  printDiedCars(update_active_cars);
  PublishActivePoint(update_active_cars);
}

std::vector<Point> LaserTracking::HistoryVecToBuffer(std::list<std::vector<Point> > history_list){
  std::vector<Point> buffer, tmp;
//  ROS_INFO("history buffer size: %d", int(tmp.size()));
  for(std::list<std::vector<Point> >::iterator it=history_list.begin(); it!=history_list.end(); it++){
    std::vector<Point> t = *it;
    for(int i=0; i<t.size(); i++){
      tmp.push_back(t[i]);
    }
  }
  ROS_INFO("temp buffer size: %d", int(tmp.size()));
  for(int i=0; i<tmp.size(); i++){
    double dist = findNearDistanceToPointVec(tmp[i], buffer);
    if(dist > camer_laser_thresh_){
      buffer.push_back(tmp[i]);
    }
  }
  return buffer;
}

void LaserTracking::ReceiveEnemies(const roborts_msgs::ArmorsPosConstPtr &msg){
  UpdateRoboState();
  std::vector<Point> received_enemies;
  std::vector<TrackingPoint> laser_to_fusion;
  if(msg->num_armor > 0){
    received_enemies.clear();
    if(msg->num_armor == 1){
      Point enemy;
      enemy.x = msg->armor_0[0];
      enemy.y = msg->armor_0[1];
      received_enemies.push_back(enemy);
      //
      TrackingPoint p;
      p.x = msg->armor_0[0];
      p.y = msg->armor_0[1];
      p.vx = 0.0;
      p.vy = 0.0;
      laser_to_fusion.push_back(p);
    }else{
      Point enemy;
      enemy.x = msg->armor_0[0];
      enemy.y = msg->armor_0[1];
      received_enemies.push_back(enemy);
      enemy.x = msg->armor_1[0];
      enemy.y = msg->armor_1[1];
      received_enemies.push_back(enemy);

      TrackingPoint p;
      p.x = msg->armor_0[0];
      p.y = msg->armor_0[1];
      p.vx = 0.0;
      p.vy = 0.0;
      laser_to_fusion.push_back(p);
      p.x = msg->armor_1[0];
      p.y = msg->armor_1[1];
      p.vx = 0.0;
      p.vy = 0.0;
      laser_to_fusion.push_back(p);
    }
  }
  ROS_INFO("========= filter died car ========");
  FusionLaserCamera(laser_to_fusion, HistoryVecToBuffer(camera_history_buffer_));
//  ROS_INFO("Update tracking ... ...");
//  UpdateTracking(received_enemies);

}

void LaserTracking::UpdateTracking(std::vector<Point> points){
  if((points.size() == 0) && (tracking_enemies_.size()==0)){
    ROS_INFO("No points to update tracking points");
    return;
  }
  if(tracking_enemies_.size()==0){
    ROS_INFO("No tracking points, add points...");
    // add tracking point
    for(int i=0; i<points.size(); i++){
      TrackingPoint tracking_point;
      tracking_point.x = points[i].x;
      tracking_point.y = points[i].y;
      tracking_point.vx = 0.0;
      tracking_point.vy = 0.0;
      tracking_enemies_.push_back(tracking_point);
      leak_detected_count_.push_back(0);
      // initial tracking path
      std::list<TrackingPoint> point_path;
      point_path.push_back(tracking_point);
      tracking_path_.push_back(point_path);
    }
  }else{
    ROS_INFO("start update tracking point...");
    switch (points.size()) {
    case 0:{
      // update leak detected
      for(std::list<int>::iterator it=leak_detected_count_.begin(); it != leak_detected_count_.end(); it++){
        *it += 1;
      }
      break;
    }
    case 1:{
      int idx = GetNearPointId(points[0], tracking_enemies_);
      std::list<TrackingPoint>::iterator it = tracking_enemies_.begin();
      std::advance(it, idx);
      double near_distance = getDistance(points[0], *it);
      if(near_distance<distance_thresh_){
        // update point
        UpdateAllLeakDetected();
        UpdatePoint(points[0], idx);
        // update tracking path
        UpdateTrackingPath(points[0], idx);
      }else{
        // update leak point
        UpdateAllLeakDetected();
        // add tracking point
        AddTrackingPoint(points[0]);
        // add tracking path
        AddTrackingPath(points[0]);
      }
    }
      break;
    case 2:{
      ROS_INFO("Detected 2 points, update ...");
      printTrackingVec(tracking_enemies_, leak_detected_count_);
      ROS_WARN("point 0: %f, %f", points[0].x, points[0].y);
      ROS_WARN("point 1: %f, %f", points[1].x, points[1].y);
      int idx_0 = GetNearPointId(points[0], tracking_enemies_);
      std::list<TrackingPoint>::iterator it = tracking_enemies_.begin();
      std::advance(it, idx_0);
      double near_distance_0 = getDistance(points[0], *it);
      int idx_1 = GetNearPointId(points[1], tracking_enemies_);
      std::advance(it, idx_1 - idx_0);
      ROS_INFO("Near point for point 1: %f, %f", it->x, it->y);
      double near_distance_1 = getDistance(points[1], *it);
      ROS_INFO("Near idx: %d, %d", idx_0, idx_1);
      ROS_INFO("Near distance: %f, %f", near_distance_0, near_distance_1);
      if(idx_0 != idx_1){
        if((near_distance_0 <= distance_thresh_) && (near_distance_1 <= distance_thresh_)){
          UpdateAllLeakDetected();
          UpdatePoint(points[0], idx_0);
          // update path
          UpdateTrackingPath(points[0], idx_0);

          UpdatePoint(points[1], idx_1);
          // update path
          UpdateTrackingPath(points[1], idx_1);
        }else if ((near_distance_0 > distance_thresh_) && (near_distance_1 <= distance_thresh_)) {
          UpdateAllLeakDetected();
          UpdatePoint(points[1], idx_1);
          UpdateTrackingPath(points[1], idx_1);
          AddTrackingPoint(points[0]);
          AddTrackingPath(points[0]);
        }else if ((near_distance_0 <= distance_thresh_) && (near_distance_1 > distance_thresh_)) {
          UpdateAllLeakDetected();
          UpdatePoint(points[0], idx_0);
          UpdateTrackingPath(points[0], idx_0);
          AddTrackingPoint(points[1]);
          AddTrackingPath(points[1]);
          ROS_WARN("Add point: x %f, y %f", points[1].x, points[1].y);
        }else{
          UpdateAllLeakDetected();
          AddTrackingPoint(points[0]);
          AddTrackingPath(points[0]);
          AddTrackingPoint(points[1]);
          AddTrackingPath(points[1]);
        }
      }else{
        if(near_distance_0 < near_distance_1){
          if((near_distance_0 <= distance_thresh_) && (near_distance_1 > distance_thresh_)){
            UpdateAllLeakDetected();
            UpdatePoint(points[0], idx_0);
            UpdateTrackingPath(points[0], idx_0);
            AddTrackingPoint(points[1]);
            AddTrackingPath(points[1]);
          }else if ((near_distance_0 <= distance_thresh_) && (near_distance_1 <= distance_thresh_)){
            UpdateAllLeakDetected();
            UpdatePoint(points[0], idx_0);
            UpdateTrackingPath(points[0], idx_0);
            // TODO find another tracking point !!!
          }else{
            UpdateAllLeakDetected();
          }
        }else{
          if((near_distance_1 <= distance_thresh_) && (near_distance_0 > distance_thresh_) ){
            UpdateAllLeakDetected();
            UpdatePoint(points[1], idx_1);
            UpdateTrackingPath(points[1], idx_1);
            AddTrackingPoint(points[0]);
            AddTrackingPath(points[0]);
          }else if((near_distance_1 <= distance_thresh_) && (near_distance_0 <= distance_thresh_)){
            UpdateAllLeakDetected();
            UpdatePoint(points[1], idx_1);
            UpdateTrackingPath(points[1], idx_1);
          }else{
            UpdateAllLeakDetected();
          }
        }
      }
      break;
    }
    default:
      break;
    } // end switch
  }
  // clear heavy leak detection
  int c = 0;
  std::vector<int> delete_vec;
  for(std::list<int>::iterator it=leak_detected_count_.begin(); it!=leak_detected_count_.end(); it++){
    if(*it > delete_count_thresh_){
      std::list<TrackingPoint>::iterator it_p = tracking_enemies_.begin();
      std::list<std::list<TrackingPoint> >::iterator it_t = tracking_path_.begin();
      std::advance(it_p, c);
      std::advance(it_t, c);
      tracking_enemies_.erase(it_p);
      tracking_path_.erase(it_t);
      delete_vec.push_back(c);
    }
    c ++;
  }
  for(int i=0; i<delete_vec.size(); i++){
    std::list<int>::iterator it = leak_detected_count_.begin();
    std::advance(it, delete_vec[i] - i);
    leak_detected_count_.erase(it);
  }
  int tracking_num = tracking_enemies_.size();
  ROS_INFO("tracking point num: %d", tracking_num);
//  UpdateTrackingWindow();
  printTrackingPath(tracking_path_);
//  PublishTrackingEnemies(tracking_enemies_);
  PublishTrackingPath(tracking_path_);
//  PublishForDecision(publish_enemies_);
}
void LaserTracking::UpdatePoint(Point point, int id){
  std::list<TrackingPoint>::iterator it_p = tracking_enemies_.begin();
  std::list<int>::iterator it_count = leak_detected_count_.begin();
  std::advance(it_p, id);
  std::advance(it_count, id);
  *it_count = 0;
  double x_pred = it_p->x + it_p->vx;
  double y_pred = it_p->y + it_p->vy;
  double x_estimate = x_pred + 0.5 * (point.x - x_pred);
  double y_estimate = y_pred + 0.5 * (point.y - y_pred);
  double vx_estimate = 0.5 * (it_p->vx) + 0.5 * (point.x - it_p->x);
  double vy_estimate = 0.5 * (it_p->vy) + 0.5 * (point.y - it_p->y);
  it_p->x = x_estimate;
  it_p->y = y_estimate;
  it_p->vx = vx_estimate;
  it_p->vy = vy_estimate;
}
void LaserTracking::UpdateAllLeakDetected(){
  for(std::list<int>::iterator it=leak_detected_count_.begin(); it!=leak_detected_count_.end(); it++){
    *it += 1;
  }
}
void LaserTracking::AddTrackingPoint(Point point){
  TrackingPoint tracking_point;
  tracking_point.x = point.x;
  tracking_point.y = point.y;
  tracking_point.vx = 0.0;
  tracking_point.vy = 0.0;
  tracking_enemies_.push_back(tracking_point);
  leak_detected_count_.push_back(0);
}
int LaserTracking::GetNearPointId(Point p, std::list<TrackingPoint> p_vec){
  std::list<TrackingPoint>::iterator it=p_vec.begin();
  double near_distance = getDistance(p, *it);
  int idx = 0;
  int count = 1;
  it++;
  for(; it!=p_vec.end(); it++){
    double current_distance = getDistance(p, *it);
    if(current_distance<near_distance){
      near_distance = current_distance;
      idx = count;
    }
    count ++;
  }
  return idx;
}
void LaserTracking::PublishTrackingEnemies(std::list<TrackingPoint> p_vec){
  roborts_msgs::ArmorsPos tracking;
  if(p_vec.size() == 0){
    tracking.num_armor = 0;
  }else if (p_vec.size() == 1) {
    tracking.num_armor = 1;
    std::vector<float> pos;
    std::list<TrackingPoint>::iterator it=p_vec.begin();
    pos.push_back(it->x);
    pos.push_back(it->y);
    tracking.armor_0 = pos;
  }else if (p_vec.size() == 2) {
    tracking.num_armor = 2;
    std::vector<float> pos;
    std::list<TrackingPoint>::iterator it=p_vec.begin();
    pos.push_back(it->x);
    pos.push_back(it->y);
    tracking.armor_0 = pos;
    it ++; pos.clear();
    pos.push_back(it->x);
    pos.push_back(it->y);
    tracking.armor_1 = pos;
  }else{
    // select little leak
    tracking.num_armor = 2;
    std::list<int> leak_detected_count_copy = leak_detected_count_;
    leak_detected_count_copy.sort();
    int tmp_0, tmp_1;
    std::list<int>::iterator it=leak_detected_count_copy.begin();
    tmp_0 = *it;
    it++;
    tmp_1 = *it;
    std::list<TrackingPoint>::iterator it_p = tracking_enemies_.begin();
    if(tmp_0 != tmp_1){
      for(it = leak_detected_count_.begin(); it != leak_detected_count_.end(); it++, it_p++){
        if(*it == tmp_0){
          std::vector<float> pos;
          pos.push_back(it_p->x);
          pos.push_back(it_p->y);
          tracking.armor_0 = pos;
        }else if (*it == tmp_1) {
          std::vector<float> pos;
          pos.push_back(it_p->x);
          pos.push_back(it_p->y);
          tracking.armor_1 = pos;
        }
      }
    }else{
      std::vector<TrackingPoint> tmp_vec;
      for(it = leak_detected_count_.begin(); it != leak_detected_count_.end(); it++, it_p++){
        if(*it == tmp_0){
          tmp_vec.push_back(*it_p);
        }
      }
      if(tmp_vec.size()<2){
        ROS_WARN("what happened?!");
        tracking.num_armor = 0;
      }else {
        std::vector<float> pos;
        pos.push_back(tmp_vec[0].x);
        pos.push_back(tmp_vec[0].y);
        tracking.armor_0 = pos;
        pos.clear();
        pos.push_back(tmp_vec[1].x);
        pos.push_back(tmp_vec[1].y);
        tracking.armor_1 = pos;
      }
    }
  }
  enemies_pub_.publish(tracking);
}
void LaserTracking::UpdateTrackingPath(Point point, int id){
  std::list<std::list<TrackingPoint> >::iterator it = tracking_path_.begin();
  std::advance(it, id);
  std::list<TrackingPoint>::iterator it_p = tracking_enemies_.begin();
  std::advance(it_p, id);
  it->push_back(*it_p);
  if(it->size() > 50){
    it->pop_front();
  }
}
void LaserTracking::AddTrackingPath(Point point){
  std::list<TrackingPoint> p_vec;
  TrackingPoint p;
  p.x = point.x;
  p.y = point.y;
  p.vx = 0.0;
  p.vy = 0.0;
  p_vec.push_back(p);
  tracking_path_.push_back(p_vec);
}
void LaserTracking::UpdateTrackingWindow(){
  for(std::list<std::list<TrackingPoint> >::iterator it=tracking_path_.begin();
      it!=tracking_path_.end(); it++){
    if(it->size() > 0){
      it->pop_front();
    }
  }
}
void LaserTracking::PublishTrackingPath(std::list<std::list<TrackingPoint> > tracking_path){
  std::vector<TrackingPoint> enemies_tracking;
  publish_enemies_.clear();
  if(tracking_path.size() == 0){
    return;
  }else if (tracking_path.size() == 1) {
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    std::vector<geometry_msgs::PoseStamped> pose_vec;
    std::list<TrackingPoint> tracking;
    std::list<std::list<TrackingPoint> >::iterator it = tracking_path.begin();
    tracking = *it;
    for(std::list<TrackingPoint>::iterator it_p=tracking.begin(); it_p!=tracking.end(); it_p++){
      pose.pose.position.x = it_p->x;
      pose.pose.position.y = it_p->y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      pose_vec.push_back(pose);
    }
    TrackingPoint end_point;
    end_point.x = pose.pose.position.x;
    end_point.y = pose.pose.position.y;
    end_point.vx = 0.0;
    end_point.vy = 0.0;
    enemies_tracking.push_back(end_point);

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    path.poses = pose_vec;
    path_0_pub_.publish(path);
  }else{
    // select top 2 size path
    nav_msgs::Path path_0, path_1;
    path_0.header.stamp = ros::Time::now();
    path_0.header.frame_id = "map";
    path_1.header.stamp = ros::Time::now();
    path_1.header.frame_id = "map";
    std::list<int> size_vec;
    for(std::list<std::list<TrackingPoint> >::iterator it = tracking_path.begin();
        it!=tracking_path.end(); it++){
      int size_i = it->size();
      size_vec.push_back(size_i);
    }
    size_vec.sort();
    size_vec.reverse();
    int tmp_0, tmp_1;
    std::list<int>::iterator it_s = size_vec.begin();
    tmp_0 = *it_s; it_s++; tmp_1 = *it_s;
    if(tmp_0 != tmp_1){
      for(std::list<std::list<TrackingPoint> >::iterator it = tracking_path.begin();
          it!=tracking_path.end(); it++){
        int size_i = it->size();
        geometry_msgs::PoseStamped pose;
        std::vector<geometry_msgs::PoseStamped> pose_vec;
        std::list<TrackingPoint> tracking = *it;
        if(size_i == tmp_0){
          // publish path 0
          pose_vec.clear();
          for(std::list<TrackingPoint>::iterator it_p=tracking.begin(); it_p!=tracking.end(); it_p++){
            pose.pose.position.x = it_p->x;
            pose.pose.position.y = it_p->y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            pose_vec.push_back(pose);
          }
          TrackingPoint end_point;
          end_point.x = pose.pose.position.x;
          end_point.y = pose.pose.position.y;
          end_point.vx = 0.0;
          end_point.vy = 0.0;
          enemies_tracking.push_back(end_point);
          path_0.poses = pose_vec;
        }
        if(size_i == tmp_1){
          // publish path 1
          pose_vec.clear();
          for(std::list<TrackingPoint>::iterator it_p=tracking.begin(); it_p!=tracking.end(); it_p++){
            pose.pose.position.x = it_p->x;
            pose.pose.position.y = it_p->y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            pose_vec.push_back(pose);
          }
          TrackingPoint end_point;
          end_point.x = pose.pose.position.x;
          end_point.y = pose.pose.position.y;
          end_point.vx = 0.0;
          end_point.vy = 0.0;
          enemies_tracking.push_back(end_point);
          path_1.poses = pose_vec;
        }
      }
    }else{
      std::vector<std::list<TrackingPoint> > path_vec;
      for(std::list<std::list<TrackingPoint> >::iterator it = tracking_path.begin();
          it!=tracking_path.end(); it++){
        int size_i = it->size();
        if(size_i == tmp_0){
          path_vec.push_back(*it);
        }
      }
      if(path_vec.size() < 2)
        ROS_ERROR("wrong path vec size!!");
      geometry_msgs::PoseStamped pose;
      std::vector<geometry_msgs::PoseStamped> pose_vec;
      std::list<TrackingPoint> tracking = path_vec[0];
      //publish path 0
      pose_vec.clear();
      for(std::list<TrackingPoint>::iterator it_p=tracking.begin(); it_p!=tracking.end(); it_p++){
        pose.pose.position.x = it_p->x;
        pose.pose.position.y = it_p->y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        pose_vec.push_back(pose);
      }
      TrackingPoint end_point;
      end_point.x = pose.pose.position.x;
      end_point.y = pose.pose.position.y;
      end_point.vx = 0.0;
      end_point.vy = 0.0;
      enemies_tracking.push_back(end_point);
      path_0.poses = pose_vec;
      //publish path 1
      tracking.clear();
      pose_vec.clear();
      tracking = path_vec[1];
      for(std::list<TrackingPoint>::iterator it_p=tracking.begin(); it_p!=tracking.end(); it_p++){
        pose.pose.position.x = it_p->x;
        pose.pose.position.y = it_p->y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        pose_vec.push_back(pose);
      }
      end_point.x = pose.pose.position.x;
      end_point.y = pose.pose.position.y;
      end_point.vx = 0.0;
      end_point.vy = 0.0;
      enemies_tracking.push_back(end_point);
      path_1.poses = pose_vec;
    }
    path_0_pub_.publish(path_0);
    path_1_pub_.publish(path_1);
  }
  publish_enemies_ = enemies_tracking;
}
void LaserTracking::PublishForDecision(std::vector<TrackingPoint> points){
  if(points.size() == 0){
     enemies_for_decision_.num_armor = 0;
  }else if (points.size() == 1){
    enemies_for_decision_.num_armor = 1;
    std::vector<float> armor;
    armor.push_back(points[0].x);
    armor.push_back(points[0].y);
    enemies_for_decision_.armor_0 = armor;
  }else if(points.size() == 2){
    enemies_for_decision_.num_armor = 2;
    std::vector<float> armor;
    armor.push_back(points[0].x);
    armor.push_back(points[0].y);
    enemies_for_decision_.armor_0 = armor;
    armor.clear();
    armor.push_back(points[1].x);
    armor.push_back(points[1].y);
    enemies_for_decision_.armor_1 = armor;
  }else{
    enemies_for_decision_.num_armor = 0;
    ROS_ERROR("points size is wrong!");
  }
  enemies_pub_.publish(enemies_for_decision_);
}
void LaserTracking::PublishActivePoint(std::vector<Point> points){
  if(points.size() == 0){
    enemies_for_decision_.num_armor = 0;
  }else if (points.size() == 1){
    enemies_for_decision_.num_armor = 1;
    std::vector<float> armor;
    armor.push_back(points[0].x);
    armor.push_back(points[0].y);
    enemies_for_decision_.armor_0 = armor;
  }else if(points.size() > 1){
    enemies_for_decision_.num_armor = 2;
    std::vector<float> armor;
    armor.push_back(points[0].x);
    armor.push_back(points[0].y);
    enemies_for_decision_.armor_0 = armor;
    armor.clear();
    armor.push_back(points[1].x);
    armor.push_back(points[1].y);
    enemies_for_decision_.armor_1 = armor;
  }
  enemies_pub_.publish(enemies_for_decision_);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_tracking");
  ros::NodeHandle nh;
  ROS_INFO("Hello world!");
  LaserTracking laser;
  ros::spin();
}
