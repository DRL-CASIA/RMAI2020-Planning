/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include "dynamic_obstacle_layer_setting.pb.h"
#include "friend_layer.h"

namespace roborts_costmap {

void FriendLayer::OnInitialize() {
  ros::NodeHandle nh;
  ParaDynamicObstacleLayer para_obstacle;

  std::string obstacle_map = ros::package::getPath("roborts_costmap") + \
      "/config/friend_layer_config.prototxt";
  roborts_common::ReadProtoFromTextFile(obstacle_map.c_str(), &para_obstacle);
  double observation_keep_time = 0.1, expected_update_rate = 10.0, min_obstacle_height = 0.2, \
 max_obstacle_height = 0.6, obstacle_range = 2.5, raytrace_range = 3.0, transform_tolerance = 0.2;
  observation_keep_time = para_obstacle.observation_keep_time();
  expected_update_rate = para_obstacle.expected_update_rate();
  transform_tolerance = para_obstacle.transform_tolerance();
  max_obstacle_height = para_obstacle.max_obstacle_height();
  min_obstacle_height = para_obstacle.min_obstacle_height();
  obstacle_range = para_obstacle.obstacle_range();
  raytrace_range = para_obstacle.raytrace_range();
  max_obstacle_height_ = max_obstacle_height;
  footprint_clearing_enabled_ = para_obstacle.footprint_clearing_enabled();
  std::string topic_string = "LaserScan", sensor_frame = "laser_frame";
  topic_string = para_obstacle.topic_string();
  sensor_frame = para_obstacle.sensor_frame();
  obstacle_size_ = para_obstacle.obstacle_size();

  bool clearing = false, marking = true;
  clearing = para_obstacle.clearing();
  marking = para_obstacle.marking();
  rolling_window_ = layered_costmap_->IsRollingWindow();
  bool track_unknown_space = layered_costmap_->IsTrackingUnknown();
  if (track_unknown_space) {
    default_value_ = NO_INFORMATION;
  } else {
    default_value_ = FREE_SPACE;
  }
  is_current_ = true;
  global_frame_ = layered_costmap_->GetGlobalFrameID();
  FriendLayer::MatchSize();
  observation_buffers_.push_back(std::shared_ptr<ObservationBuffer>(new ObservationBuffer(topic_string,
                                                                                            observation_keep_time,
                                                                                            expected_update_rate,
                                                                                            min_obstacle_height,
                                                                                            max_obstacle_height,
                                                                                            obstacle_range,
                                                                                            raytrace_range,
                                                                                            *tf_,
                                                                                            global_frame_,
                                                                                            sensor_frame,
                                                                                            transform_tolerance)));
  if (marking) {
    marking_buffers_.push_back(observation_buffers_.back());
  }
  if (clearing) {
    clearing_buffers_.push_back(observation_buffers_.back());
  } 
  reset_time_ = std::chrono::system_clock::now();
  // ROS_INFO("Friend layer register");
  std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>
  > frisub(new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh, topic_string, 50));
  std::shared_ptr<tf::MessageFilter<geometry_msgs::PoseStamped>
  > frifilter(new tf::MessageFilter<geometry_msgs::PoseStamped>(*frisub, *tf_, global_frame_, 50));
  frifilter->registerCallback(
        boost::bind(&FriendLayer::FriendCallback, this, _1, observation_buffers_.back()));
  observation_subscribers_.push_back(frisub);
  observation_notifiers_.push_back(frifilter);
  observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
  std::vector<std::string> target_frames;
  target_frames.push_back(global_frame_);
  target_frames.push_back(sensor_frame);
  observation_notifiers_.back()->setTargetFrames(target_frames);
  is_enabled_ = true;

  goal_sub_ = nh.subscribe<roborts_msgs::GlobalPlannerActionGoal>(
    "/global_planner_node_action/goal", 1, &FriendLayer::GoalCallback, this);
  // ROS_INFO("Subscribe global goal");
}

void FriendLayer::GoalCallback(const roborts_msgs::GlobalPlannerActionGoal::ConstPtr &msg) {
  ROS_INFO("Friend layer received a goal!");
  SetGoal(msg->goal.goal);
}

void FriendLayer::SetGoal(geometry_msgs::PoseStamped goal) {
  global_goal_ = goal;
  goal_updated_ = true;
}

bool FriendLayer::GetGoal(geometry_msgs::PoseStamped &goal) {
  goal = global_goal_;
  return goal_updated_;
}

void FriendLayer::FriendCallback(const geometry_msgs::PoseStamped::ConstPtr & message,
                                      const std::shared_ptr<ObservationBuffer> &buffer) {
  // ROS_INFO("Friend call back begin!");
  bool set_point = true;
  geometry_msgs::PoseStamped goal;
  bool is_update = GetGoal(goal);
  // std::cout << goal.pose.position.x << "," << goal.pose.position.y << std::endl;
  if (is_update) {
    if (abs(goal.pose.position.x-4.0)<0.1 && abs(goal.pose.position.y-0.8)<0.1) {
      set_point = false;
    }
    else if (abs(goal.pose.position.x-4.0)<0.1 && abs(goal.pose.position.y-4.2)<0.1) {
      set_point = false;
    }
  }
  Eigen::Vector2d center;
  center[0] = message->pose.position.x;
  center[1] = message->pose.position.y;
  double size_obstacle;
  if (set_point) {
    size_obstacle = obstacle_size_;
  }
  else {
    if (center[0]>3.5 && center[0]<5.0 && center[1]>0 && center[1]<1.5) {
      size_obstacle = 0.6 * obstacle_size_;
    }
    else if (center[0]>3.0 && center[0]<4.5 && center[1]>3.5 && center[1]<5.0) {
      size_obstacle = 0.6 * obstacle_size_;
    }
    else {
      size_obstacle = obstacle_size_;
    }
  }

  geometry_msgs::PoseStamped robot_map_pose;
  tf::Stamped<tf::Pose> robot_tf_pose;
  robot_tf_pose.setIdentity();
  robot_tf_pose.frame_id_ = "base_link";
  robot_tf_pose.stamp_ = ros::Time();
  try {
    geometry_msgs::PoseStamped robot_pose;
    tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
    tf_->transformPose("map", robot_pose, robot_map_pose);
  }
  catch (tf::LookupException &ex) {
    ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
  }

  const geometry_msgs::Quaternion quaternion1 = robot_map_pose.pose.orientation;
  double dy, dx;
  dy = message->pose.position.y - robot_map_pose.pose.position.y;
  dx = message->pose.position.x - robot_map_pose.pose.position.x;
  double yaw = std::atan2(dy, dx);
  const geometry_msgs::Quaternion quaternion2 = tf::createQuaternionMsgFromYaw(yaw);
  tf::Quaternion rot1, rot2;
  tf::quaternionMsgToTF(quaternion1, rot1);
  tf::quaternionMsgToTF(quaternion2, rot2);
  double angle = rot1.angleShortestPath(rot2);
  // std::cout << "angle: " << angle << std::endl;

  if (angle > 2.1) {
    std::vector<Eigen::Vector2d> vec_obstacle_center;
    vec_obstacle_center.push_back(center);
    // std::cout << center << std::endl;

    int cnt = 0;
    pcl::PointCloud<pcl::PointXYZ> new_cloud;
    new_cloud.width = 360*vec_obstacle_center.size();
    new_cloud.height = 1;
    new_cloud.points.resize(new_cloud.width * new_cloud.height);
    for (auto center:vec_obstacle_center)
    {
      for(int loop = 0; loop < 360; ++loop)
      {
        int point_num = cnt*360 + loop;
        float point_theta = 3.1415926*loop/180;
        new_cloud.points[point_num].x = cos(point_theta)*size_obstacle + center[0];
        new_cloud.points[point_num].y = sin(point_theta)*size_obstacle + center[1];
        new_cloud.points[point_num].z = 0;
      }
      cnt++;
    }
    cnt = 0;

    sensor_msgs::PointCloud2 temp_cloud;
    pcl::toROSMsg(new_cloud, temp_cloud);
    temp_cloud.header = message->header;
    // temp_cloud.header.frame_id = "map";
    // temp_cloud.header.stamp = ros::Time::now();
    // ClearMap();
    buffer->Lock();
    buffer->ClearObservation();
    buffer->BufferCloud(temp_cloud);
    buffer->Unlock();
  }
  else {
    buffer->Lock();
    buffer->ClearObservation();
    buffer->Unlock();
  }
}

void FriendLayer::UpdateBounds(double robot_x,
                                 double robot_y,
                                 double robot_yaw,
                                 double *min_x,
                                 double *min_y,
                                 double *max_x,
                                 double *max_y) {
  ClearMap();
  if (rolling_window_) {
    UpdateOrigin(robot_x - GetSizeXWorld() / 2, robot_y - GetSizeYWorld() / 2);
  } else if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - reset_time_) > std::chrono::seconds(2)){
    reset_time_ = std::chrono::system_clock::now();
    ResetMaps();
  }
  if (!is_enabled_) {
    ROS_ERROR("Friend layer is not enabled.");
    return;
  }
  UseExtraBounds(min_x, min_y, max_x, max_y);
  bool temp_is_current = true;
  std::vector<Observation> observations, clearing_observations;
  temp_is_current = temp_is_current && GetMarkingObservations(observations);
  temp_is_current = temp_is_current && GetClearingObservations(clearing_observations);
  is_current_ = temp_is_current;

  // raytrace freespace
  for (unsigned int i = 0; i < clearing_observations.size(); ++i) {
    RaytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }

  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); it++) {
    const Observation obs = *it;
    const pcl::PointCloud<pcl::PointXYZ> &cloud = *(obs.cloud_);
    double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;
    for (unsigned int i = 0; i < cloud.points.size(); ++i) {
      double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;

      // if the obstacle is too high or too far away from the robot we won't add it
      if (pz > max_obstacle_height_) {
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y)
          + (pz - obs.origin_.z) * (pz - obs.origin_.z);

      // if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_range) {
        continue;
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      if (!World2Map(px, py, mx, my)) {
        continue;
      }
      unsigned int index = GetIndex(mx, my);
      costmap_[index] = LETHAL_OBSTACLE;

      Touch(px, py, min_x, min_y, max_x, max_y);
    }
  }
  UpdateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void FriendLayer::UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
  if (!is_enabled_) {
    ROS_WARN("Friend layer is not enabled");
    return;
  }

  if (footprint_clearing_enabled_) {
    SetConvexRegionCost(transformed_footprint_, FREE_SPACE);
  }
  combination_method_ = 1;
  switch (combination_method_) {
    case 0:  // Overwrite
      UpdateOverwriteByValid(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      UpdateOverwriteByMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

void FriendLayer::Activate() {
  for (size_t i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != nullptr) {
      observation_subscribers_[i]->subscribe();
    }
  }
  for (size_t i = 0; i < observation_buffers_.size(); ++i) {
    if (observation_buffers_[i] != nullptr) {
      observation_buffers_[i]->ResetLastUpdated();
    }
  }
}

void FriendLayer::Deactivate() {
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != nullptr)
      observation_subscribers_[i]->unsubscribe();
  }
}

void FriendLayer::Reset() {
  Deactivate();
  ResetMaps();
  is_current_ = true;
  Activate();
}

bool FriendLayer::GetMarkingObservations(std::vector<Observation> &marking_observations) const {
  bool current = true;
  // get the marking observations
  for (size_t i = 0; i < marking_buffers_.size(); ++i) {
    marking_buffers_[i]->Lock();
    marking_buffers_[i]->GetObservations(marking_observations);
    current = marking_buffers_[i]->IsCurrent() && current;
    marking_buffers_[i]->Unlock();
  }
  marking_observations.insert(marking_observations.end(),
                              static_marking_observations_.begin(), static_marking_observations_.end());
  return current;
}

bool FriendLayer::GetClearingObservations(std::vector<Observation> &clearing_observations) const {
  bool current = true;
  // get the clearing observations
  for (unsigned int i = 0; i < clearing_buffers_.size(); ++i) {
    clearing_buffers_[i]->Lock();
    clearing_buffers_[i]->GetObservations(clearing_observations);
    current = clearing_buffers_[i]->IsCurrent() && current;
    clearing_buffers_[i]->Unlock();
  }
  clearing_observations.insert(clearing_observations.end(),
                               static_clearing_observations_.begin(), static_clearing_observations_.end());
  return current;
}

void FriendLayer::RaytraceFreespace(const Observation &clearing_observation,
                                      double *min_x,
                                      double *min_y,
                                      double *max_x,
                                      double *max_y) {
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  pcl::PointCloud<pcl::PointXYZ> cloud = *(clearing_observation.cloud_);

  // get the map coordinates of the origin of the sensor
  unsigned int x0, y0;
  if (!World2Map(ox, oy, x0, y0)) {
    return;
  }

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  double origin_x = origin_x_, origin_y = origin_y_;
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;

  Touch(ox, oy, min_x, min_y, max_x, max_y);

  // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
  for (unsigned int i = 0; i < cloud.points.size(); ++i) {
    double wx = cloud.points[i].x;
    double wy = cloud.points[i].y;

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the map and scale if necessary
    double a = wx - ox;
    double b = wy - oy;

    // the minimum value to raytrace from is the origin
    if (wx < origin_x) {
      double t = (origin_x - ox) / a;
      wx = origin_x;
      wy = oy + b * t;
    }
    if (wy < origin_y) {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    // the maximum value to raytrace to is the end of the map
    if (wx > map_end_x) {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y) {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    unsigned int x1, y1;

    // check for legality just in case
    if (!World2Map(wx, wy, x1, y1))
      continue;

    unsigned int cell_raytrace_range = World2Cell(clearing_observation.raytrace_range_);
    MarkCell marker(costmap_, FREE_SPACE);
    // and finally... we can execute our trace to clear obstacles along that line
    RaytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);

    UpdateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
  }
}

void FriendLayer::UpdateRaytraceBounds(double ox,
                                         double oy,
                                         double wx,
                                         double wy,
                                         double range,
                                         double *min_x,
                                         double *min_y,
                                         double *max_x,
                                         double *max_y) {
  double dx = wx - ox, dy = wy - oy;
  double full_distance = hypot(dx, dy);
  double scale = std::min(1.0, range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  Touch(ex, ey, min_x, min_y, max_x, max_y);
}

void FriendLayer::UpdateFootprint(double robot_x,
                                    double robot_y,
                                    double robot_yaw,
                                    double *min_x,
                                    double *min_y,
                                    double *max_x,
                                    double *max_y) {
  if (!footprint_clearing_enabled_)
    return;
  TransformFootprint(robot_x, robot_y, robot_yaw, GetFootprint(), transformed_footprint_);

  for (size_t i = 0; i < transformed_footprint_.size(); i++) {
    Touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

void FriendLayer::ClearMap() {
  unsigned int x_size = GetSizeXCell();
  unsigned int y_size = GetSizeYCell();
  for (unsigned int mx = 0; mx < x_size; ++mx)
  {
    for (unsigned int my = 0; my < y_size; ++my)
    {
      unsigned int index = GetIndex(mx, my);
      costmap_[index] = FREE_SPACE;
    }
  }
}

} //namespace roborts_costmap