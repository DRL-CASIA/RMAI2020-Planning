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
#ifndef ROBORTS_COSTMAP_DETECTION_LAYER_H
#define ROBORTS_COSTMAP_DETECTION_LAYER_H

#include <chrono>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include "footprint.h"
#include "map_common.h"
#include "costmap_layer.h"
#include "layered_costmap.h"
#include "observation_buffer.h"
#include "map_common.h"

#include "roborts_msgs/ArmorsPos.h"
#include "tf/transform_listener.h"
#include "math.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/point_cloud.h"

namespace roborts_costmap {

class DetectionLayer : public CostmapLayer {
 public:
  DetectionLayer() {
    costmap_ = nullptr;
  }

  virtual ~DetectionLayer() {}
  virtual void OnInitialize();
  virtual void Activate();
  virtual void Deactivate();
  virtual void Reset();
  virtual void ClearMap();
  virtual void UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void UpdateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                            double *max_x, double *max_y) override;
  void DetectionCallback(const roborts_msgs::ArmorsPos::ConstPtr &message,
                         const std::shared_ptr<ObservationBuffer> &buffer);

 protected:
  bool GetMarkingObservations(std::vector<Observation> &marking_observations) const;
  bool GetClearingObservations(std::vector<Observation> &clearing_observations) const;
  virtual void RaytraceFreespace(const Observation &clearing_observation, double *min_x, double *min_y,
                                 double *max_x, double *max_y);
  void UpdateRaytraceBounds(double ox, double oy, double wx, double wy, double range, double *min_x, double *min_y,
                            double *max_x, double *max_y);
  void UpdateFootprint(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                       double *max_x, double *max_y);
  bool footprint_clearing_enabled_, rolling_window_;
  int combination_method_;
  std::string global_frame_;
  double max_obstacle_height_;
  float obstacle_size_;
  std::vector<geometry_msgs::Point> transformed_footprint_;
  laser_geometry::LaserProjection projector_;

  std::vector<std::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;
  std::vector<std::shared_ptr<tf::MessageFilterBase> > observation_notifiers_;
  std::vector<std::shared_ptr<ObservationBuffer> > observation_buffers_;
  std::vector<std::shared_ptr<ObservationBuffer> > marking_buffers_;
  std::vector<std::shared_ptr<ObservationBuffer> > clearing_buffers_;

  std::vector<Observation> static_clearing_observations_, static_marking_observations_;
  std::chrono::system_clock::time_point reset_time_;
};

} //namespace roborts_costmap


#endif //ROBORTS_COSTMAP_DETECTION_LAYER_H
