#include "laser_detection/laser_detection.h"

void LaserDetection::LoopupFriend(){
  int blood, friSentBullet;
  float pos_x, pos_y;
  int friEnemyNums;
  float enemy_pos_x, enemy_pos_y;
  int message_length = 100;
  zmq::message_t send_message(message_length);
  snprintf((char *) send_message.data(), message_length,
           "%d %f %f %d %d %f %f",
           0, 1.0, 1.0, 1, 0, 0.0, 0.0); // faker information ! TODO !!!

  _clientSocket.send(send_message);

  zmq::message_t rec_message;
  _clientSocket.recv(&rec_message);
  std::istringstream data(static_cast<char*>(rec_message.data()));
  data>> blood >> pos_x >> pos_y >> friSentBullet >> friEnemyNums >> enemy_pos_x >> enemy_pos_y;
  friend_.x = pos_x;
  friend_.y = pos_y;
  is_received_friend_ = true;
}

void LaserDetection::LoadMap(){
  //
  ros::ServiceClient static_map_srv_ = nh.serviceClient<nav_msgs::GetMap>(static_map_topic_);
  ros::service::waitForService(static_map_topic_, -1);
  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response res;
  if(static_map_srv_.call(req,res)) {
      ROS_INFO("Received Static Map");
      cv::Mat grid_map = cv::Mat(res.map.info.height, res.map.info.width,CV_8U,
                                 const_cast<int8_t*>(&res.map.data[0]), (size_t)res.map.info.width);
      cv::Mat detected_edges;
      cv::Canny( grid_map, detected_edges, 100, 200 );
//      cv::imshow("edge_map", detected_edges);
//      cv::waitKey(0);
      std::vector<pcl::PointXYZ> edge_points;
      for(int h_i=0; h_i<detected_edges.cols; h_i++){
        for(int w_j=0; w_j<detected_edges.rows; w_j++){
          cv::Scalar pixel = detected_edges.at<uchar>(w_j, h_i);
          if(pixel.val[0]==255){
            pcl::PointXYZ p;
            p.x = res.map.info.origin.position.x + float(h_i)*res.map.info.resolution;
            p.y = res.map.info.origin.position.y + float(w_j)*res.map.info.resolution;
            p.z = 0.05;
            edge_points.push_back(p);
          }
          else {
          }
        }
      }
      pcl::PointCloud<pcl::PointXYZ> map_points;
      map_points.width = edge_points.size();
      map_points.height = 1;
      map_points.is_dense = false;
      map_points.points.resize(map_points.width * map_points.height);
      for(size_t i=0; i<edge_points.size(); i++){
        map_points.points[i] = edge_points[i];
      }
      *map_cloud = map_points;
      // for debug
//      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//      *cloud = map_points;
//      pcl::visualization::CloudViewer viewer("map points");
//      viewer.showCloud(cloud);
//      while(!viewer.wasStopped()){

//      }
    } else{
      ROS_ERROR("Get static map failed");;
      return;
    }
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_map(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(map_cloud);
  sor.setLeafSize(0.05f, 0.05f, 0.05f);
  sor.filter(*downsample_map);
  *map_cloud = *downsample_map;
  ROS_INFO("Load pointcloud map successfully!");
}

void LaserDetection::UpdateLaser(const sensor_msgs::LaserScanConstPtr &msg){
  laser_geometry::LaserProjection projector_;
  // update

  tf::StampedTransform transform_laser;
  try{
    listener_.lookupTransform(map_frame_, laser_frame_, ros::Time(0), transform_laser);
  }catch (tf::TransformException ex){
    return;
  }
  std::printf("received laser \n");
  sensor_msgs::PointCloud cloud;
  projector_.transformLaserScanToPointCloud(laser_frame_,*msg,cloud,listener_);
  pcl::PointCloud<pcl::PointXYZ> cloud_out;
  sensor_msgs::PointCloud2 pc2;
  sensor_msgs::convertPointCloudToPointCloud2(cloud, pc2);
  pcl::fromROSMsg(pc2, cloud_out);
  pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
  *laser_cloud = cloud_out;

  double roll, pitch, yaw;
  transform_laser.getBasis().getEulerYPR(yaw, pitch, roll);
  double odom_x, odom_y;
  odom_x = transform_laser.getOrigin().getX();
  odom_y = transform_laser.getOrigin().getY();
  self_pose_.x = odom_x;
  self_pose_.y = odom_y;
  Eigen::Matrix4f transform_ = Eigen::Matrix4f::Identity();
  transform_(0, 0) = cos(yaw);
  transform_(0, 1) = -sin(yaw);
  transform_(1, 0) = sin(yaw);
  transform_(1, 1) = cos(yaw);
  transform_(0, 3) = odom_x;
  transform_(1, 3) = odom_y;
  pcl::transformPointCloud(*laser_cloud, *odom_cloud, transform_);
  Detection(odom_cloud);
}

void LaserDetection::Detection(pcl::PointCloud<pcl::PointXYZ>::Ptr laser){
  // match two cloud
//  std::printf("detecting object \n");
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setTransformationEpsilon(1e-8);
  icp.setInputSource(laser);
  icp.setInputTarget(map_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sub_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f Ti_inv = Eigen::Matrix4f::Identity();
  Ti = icp.getFinalTransformation();
  Ti_inv = Ti.inverse();
  *sub_in = Final;
//  std::printf("completed icp \n");
  // outlier
  pcl::SegmentDifferences<pcl::PointXYZ> seg_sub;
  seg_sub.setInputCloud(sub_in);
  seg_sub.setTargetCloud(map_cloud);
  seg_sub.setDistanceThreshold(0.1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sub_pc(new pcl::PointCloud<pcl::PointXYZ>);
  seg_sub.segment(*sub_pc);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_pc(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*sub_pc, *outlier_pc, Ti_inv);
  // publish pc
  PublishSegmentPointcloud(outlier_pc);
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_map_pc(new pcl::PointCloud<pcl::PointXYZ>);
  if(sub_pc->points.size()>0){
    // inlier
    pcl::SegmentDifferences<pcl::PointXYZ> seg_in;
    seg_in.setInputCloud(sub_in);
    seg_in.setTargetCloud(sub_pc);
    seg_in.setDistanceThreshold(0.2);
    seg_in.segment(*in_map_pc);
    pcl::transformPointCloud(*in_map_pc, *in_map_pc, Ti_inv);
    PublishInMapPointcloud(in_map_pc);
  }else{
    pcl::transformPointCloud(*sub_in, *in_map_pc, Ti_inv);
    PublishInMapPointcloud(in_map_pc);
  }
  // cluster

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (sub_pc);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.5); // 2cm
  ec.setMinClusterSize (3);
  ec.setMaxClusterSize (200);
  ec.setSearchMethod (tree);
  ec.setInputCloud (sub_pc);
  ec.extract (cluster_indices);
  int j = 0;
  std::cout<<"detect: ";
  std::vector<Point> detected_points;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    float center_x, center_y;
    center_x= 0.0;
    center_y = 0.0;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      cloud_cluster->points.push_back(sub_pc->points[*pit]);
      center_x += sub_pc->points[*pit].x;
      center_y += sub_pc->points[*pit].y;
    }
    int cluster_size = cloud_cluster->points.size();
    center_x = center_x / float(cluster_size);
    center_y= center_y / float(cluster_size);
    if(center_x< -1 || center_y < -1 || center_x >7 || center_y > 5 ){
      continue;
    }
    Point center;
    center.x = center_x;
    center.y = center_y;
    detected_points.push_back(center);
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    std::cout<<"cluster "<<j<<" x: "<<center_x<<" y: "<<center_y<<std::endl;
    j++;
  }
  std::cout<<std::endl;
  PublishPoints(FilterFriend(detected_points));
  // end cluster
  Tracking(detected_points);
}

std::vector<Point> LaserDetection::FilterFriend(std::vector<Point> points){
  if(is_received_friend_){
    std::vector<Point> enemy_points;
    if(points.size()>0){
      for(int i=0; i<points.size(); i++){
        double dist = getPointDistance(friend_, points[i]);
        if(dist > friend_enemy_dist_thresh_)
          enemy_points.push_back(points[i]);
      }
    }
    return enemy_points;
  }else{
    return points;
  }
}

void LaserDetection::Tracking(std::vector<Point> points){
  if(points.size() == 0){
    std::printf("no point! \n");
    if(!is_initialized_tracking_){
      PublishMsg(false, estimate_pose_);
      return;
    }
    continue_no_points ++;
    if((continue_no_points < 10) && is_initialized_tracking_){
      std::printf("prediction .... \n");
      estimate_pose_.x = estimate_pose_.x + estimate_pose_.vx;
      estimate_pose_.y = estimate_pose_.y + estimate_pose_.vy;
      estimate_pose_.vx = 0.9 * estimate_pose_.vx;
      estimate_pose_.vy = 0.9 * estimate_pose_.vy;
      PublishTF(estimate_pose_);
      PublishMsg(true, estimate_pose_);
    }
    if(continue_no_points >=10){
      is_initialized_tracking_ = false;
      PublishMsg(false, estimate_pose_);
    }
    return;
  }
  if(!is_initialized_tracking_){
    // initialize tracking
    float min_d = getPointDistance(self_pose_, points[0]);
    int min_idx = 0;
    for(int i=1; i<points.size(); i++){
      float current_d = getPointDistance(self_pose_, points[i]);
      if(current_d < min_d){
        min_d = current_d;
        min_idx = i;
      }
    }
    estimate_pose_.x = points[min_idx].x;
    estimate_pose_.y = points[min_idx].y;
    estimate_pose_.vx = 0.0;
    estimate_pose_.vy = 0.0;
    is_initialized_tracking_ = true;
    continue_no_points = 0;
  }else{
    // tracking
    Point current_point, near_point;
    current_point.x = estimate_pose_.x; current_point.y = estimate_pose_.y;
    float min_distance;
    nearstPoint(current_point, points, near_point, min_distance);
    if(min_distance>0.5){
      continue_no_points ++;
      PublishMsg(false, estimate_pose_);
    }else{
      // filter
      float observe_x = near_point.x;
      float observe_y = near_point.y;
      float observe_vx = near_point.x - estimate_pose_.x;
      float observe_vy = near_point.y - estimate_pose_.y;
      float predict_x = estimate_pose_.x + estimate_pose_.vx;
      float predict_y = estimate_pose_.y + estimate_pose_.vy;
      float predict_vx = estimate_pose_.vx;
      float predict_vy = estimate_pose_.vy;
      estimate_pose_.x = predict_x + 0.6*(observe_x - predict_x);
      estimate_pose_.y = predict_y + 0.6*(observe_y - predict_y);
      estimate_pose_.vx = predict_vx + 0.1*(observe_vx - predict_vx);
      estimate_pose_.vy = predict_vy + 0.1*(observe_vy - predict_vy);
      std::cout<<"tracking: x: "<<estimate_pose_.x<<" y: "<<estimate_pose_.y<<std::endl;
      PublishTF(estimate_pose_);
      PublishMsg(true, estimate_pose_);
    }
  }
}

void LaserDetection::PublishTF(Pose pose){
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose.x, pose.y, 0.2));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  transform.setRotation(q);
  enemy_tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", laser_tracking_frame_));
}

void LaserDetection::PublishMsg(bool is_valid, Pose pose){
  roborts_msgs::ArmorPos laser_enemy;
  laser_enemy.is_valid = is_valid;
  laser_enemy.x = pose.x;
  laser_enemy.y = pose.y;
  laser_enemy.z = 0.2;
  double distance = sqrt((self_pose_.x - pose.x) * (self_pose_.x - pose.x) +
                         (self_pose_.y - pose.y) * (self_pose_.y - pose.y));
  laser_enemy.distance = distance;
  tf::StampedTransform transform_laser;
  try{
    listener_.lookupTransform(gimbal_link_frame_, laser_tracking_frame_, ros::Time(0), transform_laser);
    std::vector<double> pose_in_gimbal;
    pose_in_gimbal.push_back(transform_laser.getOrigin().x());
    pose_in_gimbal.push_back(transform_laser.getOrigin().y());
    pose_in_gimbal.push_back(transform_laser.getOrigin().z());
    laser_enemy.pose_in_gimbal = pose_in_gimbal;
  }catch (tf::TransformException ex){
    //    return;
  }
  std::vector<double> motion_vec;
  motion_vec.push_back(pose.vx);
  motion_vec.push_back(pose.vy);
  laser_enemy.motion_vec = motion_vec;
  enemy_pub_.publish(laser_enemy);
}

void LaserDetection::PublishPoints(std::vector<Point> points){
  roborts_msgs::ArmorsPos enemy;
  enemy.num_armor = points.size();
  if(enemy.num_armor>2){
    enemy.num_armor = 2;
  }
  ROS_WARN("Detected enemy num: %d", enemy.num_armor);
  if(points.size() == 1){
    std::vector<float> armor_0;
    armor_0.push_back(points[0].x);
    armor_0.push_back(points[0].y);
    enemy.armor_0 = armor_0;
    last_detected_enemies_.armor_0 = armor_0;
    ROS_WARN("Enemy 0 x: %f, y: %f", points[0].x, points[0].y);
  }
  if(points.size()>1){
    std::vector<float> armor_0;
    armor_0.push_back(points[0].x);
    armor_0.push_back(points[0].y);
    enemy.armor_0 = armor_0;
    last_detected_enemies_.armor_0 = armor_0;
    ROS_WARN("Enemy 0 x: %f, y: %f", points[0].x, points[0].y);
    std::vector<float> armor_1;
    armor_1.push_back(points[1].x);
    armor_1.push_back(points[1].y);
    enemy.armor_1 = armor_1;
    last_detected_enemies_.armor_1 = armor_1;
    ROS_WARN("Enemy 1 x: %f, y: %f", points[1].x, points[1].y);
  }
  enemy.armor_0 = last_detected_enemies_.armor_0;
  enemy.armor_1 = last_detected_enemies_.armor_1;
  points_pub_.publish(enemy);
}

void LaserDetection::PublishSegmentPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr seg_laser){
  sensor_msgs::PointCloud2Ptr pub_seg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*seg_laser, *pub_seg);
  pub_seg->header.frame_id = "map";
  pub_seg->header.stamp = ros::Time::now();
  seg_pc_pub_.publish(*pub_seg);
}
void LaserDetection::PublishInMapPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr seg_laser){
  sensor_msgs::PointCloud2Ptr pub_seg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*seg_laser, *pub_seg);
  pub_seg->header.frame_id = "map";
  pub_seg->header.stamp = ros::Time::now();
  in_map_pub_.publish(*pub_seg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_detection");
  LaserDetection laser_detection;
//  ros::Rate loop(20);
//  while(ros::ok()){
//    laser_detection.LoopupFriend();
//    loop.sleep();
//    ros::spinOnce();
//  }
  ros::spin();
  return 0;
}
