#ifndef ROBORTS_DECISION_ATTACK_BEHAVIOR_H
#define ROBORTS_DECISION_ATTACK_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class AttackBehavior {
 public:
  AttackBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {

    auto executor_state = Update();


    // directly to enemy
    if (executor_state != BehaviorState::RUNNING){
        geometry_msgs::PoseStamped enemy_pose = blackboard_->GetEnemy();
        chassis_executor_->Execute(enemy_pose);
        blackboard_->SetMyGoal(enemy_pose);
       

    }

    
    // // for initialize or aborted
    // if (executor_state != BehaviorState::RUNNING) {
    //   // obtain initialize value
    //   geometry_msgs::PoseStamped robot_map_pose = blackboard_->GetRobotMapPose();
    //   geometry_msgs::PoseStamped enemy_map_pose = blackboard_->GetEnemy();
       
     

    //   double goal_x, goal_y;
    //   std::vector<geometry_msgs::PoseStamped> can_goals;
      
    //   // obtain can goals
    //   double enemy_x, enemy_y;
    //   enemy_x = enemy_map_pose.pose.position.x;
    //   enemy_y = enemy_map_pose.pose.position.y;
    
    //     for (double x=-2; x<=2; x += 0.1){
    //         for (double y=-2; y<=2; y += 0.1){
    //             if ( (x!=0 || y !=0 ) && (std::abs(x)>=1.0)  && std::abs(y) >= 1.0){
    //                 double x_ = enemy_map_pose.pose.position.x + x;
    //                 double y_ = enemy_map_pose.pose.position.y + y;


    //                 // opp reload and shield
    //                 if (blackboard_->GetEulerDistance(x_,y_, oppReload.pose.position.x, oppReload.pose.position.y) <= 1.0
    //                 || blackboard_->GetEulerDistance(x_,y_, oppShield.pose.position.x, oppShield.pose.position.y) <= 1.0)
    //                     continue;

    //                 // out of boundary
    //                 if (x_<0 || x_>8 || y_<0 || y_>5) continue;

    //                 unsigned int start_x, start_y, end_x, end_y;
    //                 blackboard_->GetCostMap2D()->World2Map(x_, y_, start_x, start_y);
    //                 blackboard_->GetCostMap2D()->World2Map(enemy_x, enemy_y, end_x, end_y);
    //                 int x0, y0, x1,y1;
    //                 x0 = std::min(start_x, end_x);
    //                 y0 = std::min(start_y, end_y);
    //                 x1 = std::max(start_x, end_x);
    //                 y1 = std::max(start_y, end_y);
                
    //                 // set up
    //                 bool flag = false;
    //                 for (int i=x0; i<=x1; i++){
    //                     for (int j=y0; j<=y1; j++){
    //                         if (blackboard_->GetCostMap2D()->GetCost(i,j)>253){
    //                             flag = true;
    //                             break;
    //                         }
                                
    //                     }
    //                     if (flag) break;
    //                 }
    //                 if (flag) continue;
    //                 geometry_msgs::PoseStamped pose;
    //                 pose.pose.position.x = x_;
    //                 pose.pose.position.y = y_;
    //                 can_goals.push_back(pose);
    //             }
                
    //         }
    //     }
         
    
      
      
    //   double min_distance = 10.0;  // 10m
    //   bool flag = false;
    //   geometry_msgs::PoseStamped goal;
    //   for (int i=0; i<can_goals.size(); i++){
    //       double dist =  blackboard_->GetDistance(can_goals[i], robot_map_pose);
    //       if (dist < min_distance && !blackboard_->IsBombAllyGoal(can_goals[i])){
    //           min_distance = dist;
    //           goal = can_goals[i];
    //           flag = true;
              
    //       }
    //   }
    //   if (flag && min_distance >= blackboard_->threshold.near_dist){
    //           goal = _GoalToAttack(goal, enemy_map_pose);
    //           chassis_executor_->Execute(goal);
    //           blackboard_->SetMyGoal(goal); 
    //   }
    //   else if (flag){
    //         goal = _GoalToAttack(robot_map_pose, enemy_map_pose);
    //         chassis_executor_->Execute(goal);
    //         blackboard_->SetMyGoal(goal);
    //         // printf("Set Turn Angle!\n");  
    //   }
     
           
      
    // }
    // // else // for unwalkable area
    // //     if (executor_state == BehaviorState::RUNNING){
        
    // //         geometry_msgs::PoseStamped goal = blackboard_->GetMyGoal();
    // //         double gx = goal.pose.position.x;
    // //         double gy = goal.pose.position.y;
    // //         printf("Attack gx,gy (%f, %f) ", gx, gy);
    // //         if (gx <=0 || gx>=8 || gy<=0 || gy>=5){
    // //             Cancel();
    // //         }
    // //         else{
    // //             unsigned int mx, my;
    // //             blackboard_->GetCostMap2D()->World2Map(gx, gy, mx, my);
    // //             unsigned char cost = blackboard_->GetCostMap2D()->GetCost(mx,my);
    // //             printf("cost %d\n", cost);
    // //             if (cost>=253)  {
    // //                 Cancel();
    // //             }

    // //         }
            

        
    // // }

  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }



  ~AttackBehavior() = default;

 private:

  geometry_msgs::PoseStamped _GoalToAttack(geometry_msgs::PoseStamped us, geometry_msgs::PoseStamped enemy){
       geometry_msgs::PoseStamped goal;
       goal.pose.orientation = blackboard_->GetRelativeQuaternion(enemy, us);
       goal.pose.position.x = us.pose.position.x;
       goal.pose.position.y = us.pose.position.y;
       goal.pose.position.z = 0;
       return goal;
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    oppShield.header.frame_id = "map";
    oppReload.header.frame_id = "map";
    if (decision_config.isblue()){
        oppShield.pose.position.x = decision_config.red().shield_point().x();
        oppShield.pose.position.y = decision_config.red().shield_point().y();
        oppShield.pose.position.z = decision_config.red().shield_point().z();
        oppShield.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                                                                decision_config.red().shield_point().roll(),
                                                                decision_config.red().shield_point().pitch(),
                                                                decision_config.red().shield_point().yaw()); 
        oppReload.pose.position.x = decision_config.red().reload_point().x();
        oppReload.pose.position.y = decision_config.red().reload_point().y();
        oppReload.pose.position.z = decision_config.red().reload_point().z();
        oppReload.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                                                                decision_config.red().reload_point().roll(),
                                                                decision_config.red().reload_point().pitch(),
                                                                decision_config.red().reload_point().yaw()); 
        
    }
    else{
        oppShield.pose.position.x = decision_config.blue().shield_point().x();
        oppShield.pose.position.y = decision_config.blue().shield_point().y();
        oppShield.pose.position.z = decision_config.blue().shield_point().z();
        oppShield.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                                                                decision_config.blue().shield_point().roll(),
                                                                decision_config.blue().shield_point().pitch(),
                                                                decision_config.blue().shield_point().yaw()); 
        oppReload.pose.position.x = decision_config.blue().reload_point().x();
        oppReload.pose.position.y = decision_config.blue().reload_point().y();
        oppReload.pose.position.z = decision_config.blue().reload_point().z();
        oppReload.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                                                                decision_config.blue().reload_point().roll(),
                                                                decision_config.blue().reload_point().pitch(),
                                                                decision_config.blue().reload_point().yaw()); 
        
    }
    

    return true;
  }



  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! attack goal
  std::vector<geometry_msgs::PoseStamped> ambush_goals_;
  

  // opp shield and reload
  geometry_msgs::PoseStamped oppShield, oppReload;


};
}

#endif //ROBORTS_DECISION_ATTACK_BEHAVIOR_H
