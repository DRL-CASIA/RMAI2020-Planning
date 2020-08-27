#ifndef ROBORTS_DECISION_RELOAD_BEHAVIOR_H
#define ROBORTS_DECISION_RELOAD_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class ReloadBehavior {
 public:
  ReloadBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard),
                                                       count(0),
                                                       count_limit(60) {

    start_supplying = false;
    reload_position_.header.frame_id = "map";
    reload_position_.pose.orientation.x = 0;
    reload_position_.pose.orientation.y = 0;
    reload_position_.pose.orientation.z = 0;
    reload_position_.pose.orientation.w = 1;

    reload_position_.pose.position.x = 0;
    reload_position_.pose.position.y = 0;
    reload_position_.pose.position.z = 0;
    supply_bullet = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {
  
    auto executor_state = Update();

    auto robot_map_pose = blackboard_->GetRobotMapPose();
    auto dx = reload_position_.pose.position.x - robot_map_pose.pose.position.x;
    auto dy = reload_position_.pose.position.y - robot_map_pose.pose.position.y;


    auto d_yaw =  blackboard_->GetAngle(reload_position_, robot_map_pose);
    geometry_msgs::PoseStamped enemy;
       
    if (executor_state != BehaviorState::RUNNING) {
      if ((std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2) 
         && !blackboard_->IsBombAllyGoal(reload_position_)
         && !start_supplying) 
      {
          chassis_executor_->Execute(reload_position_);
          blackboard_->SetMyGoal(reload_position_);
      }
    //   // turn towards enemy when load points
    //   else if (start_supplying){
    //       if (blackboard_->info.has_my_enemy){
    //            enemy = blackboard_->GetEnemy();
    //            robot_map_pose.pose.orientation = blackboard_->GetRelativeQuaternion(enemy, robot_map_pose);
    //       }
    //       else{
    //           robot_map_pose.pose.orientation = blackboard_->GetRelativeQuaternion(blackboard_->info.opp_reload, blackboard_->info.my_reload);
    //       }
    //       chassis_executor_->Execute(robot_map_pose);
    //       blackboard_->SetMyGoal(robot_map_pose);

    //   }
    }
          
           
          

    // add ambush when I have bullet
    if ( (blackboard_->info.remain_bullet >0)  || 
         (!blackboard_->hasOtherUnitsInThisArea(ambush_pose)  && blackboard_->GetDistance(blackboard_->info.ally, reload_position_) <= blackboard_->threshold.near_dist) ){
        if (blackboard_->GetDistance(robot_map_pose, ambush_pose) >= blackboard_->threshold.near_dist){
             if (blackboard_->info.has_my_enemy){
                enemy = blackboard_->GetEnemy();
                ambush_pose.pose.orientation = blackboard_->GetRelativeQuaternion(enemy, ambush_pose);
             }
             else{
                ambush_pose.pose.orientation = blackboard_->GetRelativeQuaternion(blackboard_->info.opp_reload, blackboard_->info.my_reload);
             }
             chassis_executor_->Execute(ambush_pose);
             blackboard_->SetMyGoal(ambush_pose);
        }
        else{
            if (blackboard_->info.has_my_enemy){
                enemy = blackboard_->GetEnemy();
                robot_map_pose.pose.orientation = blackboard_->GetRelativeQuaternion(enemy, robot_map_pose);
            }
            else{
                robot_map_pose.pose.orientation = blackboard_->GetRelativeQuaternion(blackboard_->info.opp_reload, blackboard_->info.my_reload);
            }         
            chassis_executor_->Execute(robot_map_pose);
            blackboard_->SetMyGoal(robot_map_pose);
        }
    }

    

    

    // start reload
    if (std::sqrt(std::pow(dx,2) + std::pow(dy,2))<=0.15  && count==0){
        if (blackboard_->info.remain_hp >= blackboard_->info.ally_remain_hp + 200  && blackboard_->info.times_to_supply>=2) supply_bullet = 100;
        else  supply_bullet = 50;
        blackboard_->SupplyRequest(supply_bullet);
        start_supplying = true;
        new_thread = new std::thread(boost::bind(& ReloadBehavior::CountLoop, this));
    }

    // finish reload
    if (count >= count_limit || ( start_supplying && !blackboard_->info.is_supplying)){
        std::cout<<"Reload is finised!" << std::endl;
        if (supply_bullet==50)  blackboard_->info.times_to_supply -= 1;  
        else blackboard_->info.times_to_supply = 0;
        blackboard_->info.remain_bullet = supply_bullet;
        new_thread->join();
        start_supplying = false;
        blackboard_->info.is_supplying = false;
        count = 0;
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
    count = 0;
    start_supplying = false;

  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    reload_position_.header.frame_id = "map";
    int last_idx =  decision_config.blue().patrol().size() - 1;
    if (decision_config.isblue()){
        reload_position_.pose.position.x = decision_config.blue().reload_point().x();
        reload_position_.pose.position.z = decision_config.blue().reload_point().z();
        reload_position_.pose.position.y = decision_config.blue().reload_point().y();
        reload_position_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                                                                decision_config.blue().reload_point().roll(),
                                                                decision_config.blue().reload_point().pitch(),
                                                                decision_config.blue().reload_point().yaw()); 
        ambush_pose = blackboard_->Point2PoseStamped(decision_config.blue().patrol(last_idx));
        
    }
    else{
        reload_position_.pose.position.x = decision_config.red().reload_point().x();
        reload_position_.pose.position.z = decision_config.red().reload_point().z();
        reload_position_.pose.position.y = decision_config.red().reload_point().y();
        reload_position_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                                                                decision_config.red().reload_point().roll(),
                                                                decision_config.red().reload_point().pitch(),
                                                                decision_config.red().reload_point().yaw());
        ambush_pose = blackboard_->Point2PoseStamped(decision_config.red().patrol(last_idx -1));
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

  ~ReloadBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! boot position
  geometry_msgs::PoseStamped reload_position_;

  //! chase buffer
  std::vector<geometry_msgs::PoseStamped> chase_buffer_;
  unsigned int chase_count_;
  unsigned int supply_bullet;

  int count, count_limit;
  std::thread * new_thread;
  geometry_msgs::PoseStamped ambush_pose;
  bool start_supplying;
  bool dodge_in_reload;

};
}


#endif //ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H
