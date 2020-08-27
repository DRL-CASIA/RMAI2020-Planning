#ifndef ROBORTS_DECISION_SEARCH_BEHAVIOR_H
#define ROBORTS_DECISION_SEARCH_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class SearchBehavior {
 public:
  SearchBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {


    last_position_.header.frame_id = "map";
    last_position_.pose.orientation.x = 0;
    last_position_.pose.orientation.y = 0;
    last_position_.pose.orientation.z = 0;
    last_position_.pose.orientation.w = 1;

    last_position_.pose.position.x = 0;
    last_position_.pose.position.y = 0;
    last_position_.pose.position.z = 0;

    search_index_ = 0;
    search_count_ = 0;
    SearchBegin = true;
    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {

    auto executor_state = Update();
    // infos 
    int mx_, my_;
    blackboard_->GetCostMap2D()->World2MapWithBoundary(blackboard_->info.my_goal.pose.position.x, blackboard_->info.my_goal.pose.position.y, mx_, my_);

    if (blackboard_->GetCostMap2D()->GetCost(mx_,my_)>=253  && executor_state == BehaviorState::RUNNING){
        chassis_executor_->Cancel();
        search_index_ = (search_index_ + 1) % search_region_1_.size();

    }

    if (executor_state != BehaviorState::RUNNING) {
      // got enemy
      if (blackboard_->info.got_last_enemy){
          geometry_msgs::PoseStamped last_enemy = blackboard_->GetEnemy();
          blackboard_->info.got_last_enemy = false;
          if (!blackboard_->IsBombAllyGoal(last_enemy)){
                chassis_executor_->Execute(last_enemy);
                blackboard_->SetMyGoal(last_enemy);
          }
      }
      else if (SearchBegin){
          SearchBegin = false;
          geometry_msgs::PoseStamped my_pose = blackboard_->GetRobotMapPose();
          double min_distance = 100;
          double distance;
          for (int i=0; i<search_region_1_.size(); i++){
            distance = blackboard_->GetDistance(my_pose, search_region_1_[i]);
            if (min_distance>distance){
                min_distance = distance;
                search_index_ = i;
            }
          }
          geometry_msgs::PoseStamped goal = search_region_1_[search_index_];
          if (!blackboard_->IsBombAllyGoal(goal)){
              chassis_executor_->Execute(goal);
              blackboard_->SetMyGoal(goal);
          }
          search_index_ = (search_index_ + 1) % search_region_1_.size();
          

      }
      // search to find enemy
      else{
          geometry_msgs::PoseStamped goal = search_region_1_[search_index_];
          if (!blackboard_->IsBombAllyGoal(goal)){
              chassis_executor_->Execute(goal);
              blackboard_->SetMyGoal(goal);
             
          }
           search_index_ = (search_index_ + 1) % search_region_1_.size();
          
      }


      
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
    SearchBegin = true;
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }
     
    for (int i=0; i<decision_config.search_region_1().size();i++)
        search_region_1_.push_back(blackboard_->Point2PoseStamped(decision_config.search_region_1(i)));

    // may have more efficient way to search a region(enemy where disappear)
    // search_region_.resize((unsigned int)(decision_config.search_region_1().size()));

    // for (int i = 0; i != decision_config.search_region_1().size(); i++) {
    //   geometry_msgs::PoseStamped search_point;
    //   search_point.header.frame_id = "map";
    //   search_point.pose.position.x = decision_config.search_region_1(i).x();
    //   search_point.pose.position.y = decision_config.search_region_1(i).y();
    //   search_point.pose.position.z = decision_config.search_region_1(i).z();

    //   auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_1(i).roll(),
    //                                                             decision_config.search_region_1(i).pitch(),
    //                                                             decision_config.search_region_1(i).yaw());
    //   search_point.pose.orientation = quaternion;
    //   search_region_1_.push_back(search_point);
    // }

    // for (int i = 0; i != decision_config.search_region_2().size(); i++) {
    //   geometry_msgs::PoseStamped search_point;
    //   search_point.header.frame_id = "map";
    //   search_point.pose.position.x = decision_config.search_region_2(i).x();
    //   search_point.pose.position.y = decision_config.search_region_2(i).y();
    //   search_point.pose.position.z = decision_config.search_region_2(i).z();

    //   auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_2(i).roll(),
    //                                                             decision_config.search_region_2(i).pitch(),
    //                                                             decision_config.search_region_2(i).yaw());
    //   search_point.pose.orientation = quaternion;
    //   search_region_2_.push_back(search_point);
    // }

    // for (int i = 0; i != decision_config.search_region_3().size(); i++) {
    //   geometry_msgs::PoseStamped search_point;
    //   search_point.header.frame_id = "map";
    //   search_point.pose.position.x = decision_config.search_region_3(i).x();
    //   search_point.pose.position.y = decision_config.search_region_3(i).y();
    //   search_point.pose.position.z = decision_config.search_region_3(i).z();

    //   auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_3(i).roll(),
    //                                                             decision_config.search_region_3(i).pitch(),
    //                                                             decision_config.search_region_3(i).yaw());
    //   search_point.pose.orientation = quaternion;
    //   search_region_3_.push_back(search_point);
    // }

    // for (int i = 0; i != decision_config.search_region_4().size(); i++) {
    //   geometry_msgs::PoseStamped search_point;
    //   search_point.header.frame_id = "map";
    //   search_point.pose.position.x = decision_config.search_region_4(i).x();
    //   search_point.pose.position.y = decision_config.search_region_4(i).y();
    //   search_point.pose.position.z = decision_config.search_region_4(i).z();

    //   auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_4(i).roll(),
    //                                                             decision_config.search_region_4(i).pitch(),
    //                                                             decision_config.search_region_4(i).yaw());
    //   search_point.pose.orientation = quaternion;
    //   search_region_4_.push_back(search_point);
    // }
    return true;
  }

  void SetLastPosition(geometry_msgs::PoseStamped last_position) {
    last_position_ = last_position;
    search_count_ = 5;
  }

  ~SearchBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! chase goal
  geometry_msgs::PoseStamped last_position_;

  //! search buffer
  std::vector<geometry_msgs::PoseStamped> search_region_1_;
  std::vector<geometry_msgs::PoseStamped> search_region_2_;
  std::vector<geometry_msgs::PoseStamped> search_region_3_;
  std::vector<geometry_msgs::PoseStamped> search_region_4_;
  std::vector<geometry_msgs::PoseStamped> search_region_;
  unsigned int search_count_;
  unsigned int search_index_;
  bool SearchBegin;

};
}

#endif //ROBORTS_DECISION_SEARCH_BEHAVIOR_H
