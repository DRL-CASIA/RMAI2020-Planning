#ifndef ROBORTS_DECISION_AMBUSH_BEHAVIOR_H
#define ROBORTS_DECISION_AMBUSH_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class AmbushBehavior {
 public:
  AmbushBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {


    cancel_goal_ = true;
    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }
  }

  void Run() {

    auto executor_state = Update();

    geometry_msgs::PoseStamped my = blackboard_->GetRobotMapPose();
    geometry_msgs::PoseStamped enemy_map_pose = blackboard_->GetEnemy();


    if ((blackboard_->info.has_my_enemy || blackboard_->info.valid_front_camera_armor) 
        && blackboard_->GetDistance(my, enemy_map_pose)<=2 
        && blackboard_->GetDistance(my, enemy_map_pose)>=0.75 
        && blackboard_->GetDistance(my, blackboard_->info.opp_reload) >= 0.65
        && blackboard_->GetDistance(my, blackboard_->info.opp_shield) >= 0.65
        && blackboard_->GetDistance(my, blackboard_->info.my_reload) >= 0.65
        )
    {
        my.pose.orientation = blackboard_->GetRelativeQuaternion(enemy_map_pose, my);
        chassis_executor_->Execute(my);
        blackboard_->SetMyGoal(my);
        return;
    }

    // for initialize or aborted
    if (executor_state != BehaviorState::RUNNING) {   
      geometry_msgs::PoseStamped ally_map_pose = blackboard_->info.ally;
    

      // find out the nearest ambush point
      double min_distance = 9999999;
      double min_i = -1;
      double e_dist;
      double ally_dist;
      for (int i=0; i<point_size_; i++){
          e_dist = blackboard_->GetDistance(ambush_goals_[i], enemy_map_pose);
          ally_dist = blackboard_->GetDistance(ambush_goals_[i], ally_map_pose);
          if (e_dist < min_distance 
              && !blackboard_->IsBombAllyGoal(ambush_goals_[i])
              && ally_dist >= 0.6
              && e_dist >= 1.0
              // && distance > blackboard_->threshold.near_dist
          ){
              min_distance = e_dist;
              min_i = i;
              
          }
      }

      
      if (min_i >=0){
        // send goal to the ambush
        auto sel_goal= ambush_goals_[min_i];
        sel_goal.pose.orientation = blackboard_->GetRelativeQuaternion(enemy_map_pose, sel_goal);
        chassis_executor_->Execute(sel_goal);
        blackboard_->SetMyGoal(sel_goal);
        cancel_goal_ = true;
        
      }
      else{
         // send goal to the ambush
        auto sel_goal= my;
        sel_goal.pose.orientation = blackboard_->GetRelativeQuaternion(enemy_map_pose, sel_goal);
        chassis_executor_->Execute(sel_goal);
        blackboard_->SetMyGoal(sel_goal);
        cancel_goal_ = true;
          
      }
      
      // else if (cancel_goal_){
      //     Cancel();
      //     cancel_goal_ = false;
      // }
      
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }



  ~AmbushBehavior() = default;

 private:
  // load patrol as ambush points
  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    point_size_ = (unsigned int)(decision_config.blue().patrol().size());
    ambush_goals_.resize(point_size_);
    for (int i = 0; i != point_size_; i++) {
      if (blackboard_->info.team_blue){
          ambush_goals_[i] = blackboard_->Point2PoseStamped(decision_config.blue().patrol(i));
      }
      else{
          ambush_goals_[i] = blackboard_->Point2PoseStamped(decision_config.red().patrol(i));
      }
      
    }

    return true;
  }



  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! attack goal
  std::vector<geometry_msgs::PoseStamped> ambush_goals_;

  int point_size_;
  //! cancel flag
  bool cancel_goal_;
};
}

#endif //ROBORTS_DECISION_AMBUSH_BEHAVIOR_H
