#ifndef ROBORTS_DECISION_SHIELD_BEHAVIOR_H
#define ROBORTS_DECISION_SHIELD_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class ShieldBehavior {
 public:
  ShieldBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard),
                                                       count(0),
                                                       count_limit(100) {


    shield_position_.header.frame_id = "map";
    shield_position_.pose.orientation.x = 0;
    shield_position_.pose.orientation.y = 0;
    shield_position_.pose.orientation.z = 0;
    shield_position_.pose.orientation.w = 1;

    shield_position_.pose.position.x = 0;
    shield_position_.pose.position.y = 0;
    shield_position_.pose.position.z = 0;

    starting_buff = false;
    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {
    
    auto executor_state = Update();

    auto robot_map_pose = blackboard_->GetRobotMapPose();
    auto dx = shield_position_.pose.position.x - robot_map_pose.pose.position.x;
    auto dy = shield_position_.pose.position.y - robot_map_pose.pose.position.y;

    auto boot_yaw = tf::getYaw(shield_position_.pose.orientation);
    auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);

    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(shield_position_.pose.orientation, rot1);
    tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
    auto d_yaw =  rot1.angleShortestPath(rot2);

    if (executor_state != BehaviorState::RUNNING) {
      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2
        && !blackboard_->IsBombAllyGoal(shield_position_)
        && !starting_buff) {
          chassis_executor_->Execute(shield_position_);
          blackboard_->SetMyGoal(shield_position_);
      }
      else if (starting_buff){
           if (blackboard_->info.has_my_enemy || blackboard_->info.valid_front_camera_armor){
               geometry_msgs::PoseStamped enemy = blackboard_->GetEnemy();
               robot_map_pose.pose.orientation = blackboard_->GetRelativeQuaternion(enemy, robot_map_pose);
               chassis_executor_->Execute(robot_map_pose);
               blackboard_->SetMyGoal(robot_map_pose);
           }
      }  
    }

    if (std::sqrt(std::pow(dx,2) + std::pow(dy,2))<= blackboard_->threshold.near_dist  && count==0){
        new_thread = new std::thread(boost::bind(& ShieldBehavior::CountLoop, this));
        starting_buff = true;
        blackboard_->info.is_shielding = true;
    }

    if (count >= count_limit || blackboard_->info.has_buff){
        std::cout<<"Shield is finised!" << std::endl;
        blackboard_->info.has_buff = true;
        blackboard_->info.times_to_buff = 0;
        blackboard_->info.is_shielding = false;
        starting_buff = false;
        new_thread->join();
        count = 0;
    }
  }

  void Cancel() {
      chassis_executor_->Cancel();
      starting_buff = false;
      blackboard_->info.is_shielding = false;
      count = 0;
    
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    shield_position_.header.frame_id = "map";
    if (decision_config.isblue()){
        shield_position_.pose.position.x = decision_config.blue().shield_point().x();
        shield_position_.pose.position.z = decision_config.blue().shield_point().z();
        shield_position_.pose.position.y = decision_config.blue().shield_point().y();
        shield_position_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                                                                decision_config.blue().shield_point().roll(),
                                                                decision_config.blue().shield_point().pitch(),
                                                                decision_config.blue().shield_point().yaw());  
        
    }
    else{
        shield_position_.pose.position.x = decision_config.red().shield_point().x();
        shield_position_.pose.position.z = decision_config.red().shield_point().z();
        shield_position_.pose.position.y = decision_config.red().shield_point().y();
        shield_position_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                                                                decision_config.red().shield_point().roll(),
                                                                decision_config.red().shield_point().pitch(),
                                                                decision_config.red().shield_point().yaw());
    }
    

    return true;
  }


  void CountLoop(){
     ros::Rate loop(10);
     while (count < count_limit){
        count++;
        loop.sleep();
     }  
  }

  ~ShieldBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! boot position
  geometry_msgs::PoseStamped shield_position_;

 

  int count, count_limit;
  bool starting_buff;
  std::thread * new_thread;

};
}


#endif //ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H
