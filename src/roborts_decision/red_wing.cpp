#include <ros/ros.h>

#include "executor/chassis_executor.h"


#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/reload_behavior.h"
#include "example_behavior/shield_behavior.h"
#include "example_behavior/test_behavior.h"
#include "example_behavior/ambush_behavior.h"
#include "example_behavior/attack_behavior.h"

enum BehaviorStateEnum{
     INIT = -1,
     BACKBOOT = 0,
     CHASE=1,
     SEARCH=2,
     ESCAPE=3,
     PATROL=4,
     RELOAD=5,
     SHIELD=6,
     AMBUSH=7,
     ATTACK=8,

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "red_wing");
  
  
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/red_wing.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);

  // Behavior State Enum
  BehaviorStateEnum last_state, cur_state;
  last_state = BehaviorStateEnum::INIT;
  cur_state = BehaviorStateEnum::INIT;

  roborts_decision::BackBootAreaBehavior back_boot_area_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::ChaseBehavior        chase_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::SearchBehavior       search_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::EscapeBehavior       escape_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::PatrolBehavior       patrol_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::GoalBehavior       goal_behavior(chassis_executor, blackboard);
  roborts_decision::ReloadBehavior     reload_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::ShieldBehavior     shield_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::TestBehavior      test_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::AmbushBehavior    ambush_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::AttackBehavior    attack_behavior(chassis_executor, blackboard, full_path);
  ros::Rate rate(10);

  // for filter noise command
  unsigned int count=0;
  const unsigned int count_bound = 3;
  while(ros::ok()){
    ros::spinOnce();

    // shoot and dodge command when game is on!
    if (last_state != BehaviorStateEnum::ESCAPE){
        
        if (blackboard->CanDodge()) {blackboard->StartDodge(); printf("In Dodge!\n");}
        else printf("Not In Dodge!\n");
    }
    if (blackboard->CanShoot()) blackboard->Shoot(blackboard->info.shoot_hz);
    // my pose
    geometry_msgs::PoseStamped mypose = blackboard->GetRobotMapPose();
    
    // Defense with buff----------------------------------------------------------------------------------------------------------------
    if (blackboard->info.strategy == "go_buff"){
            // state decision behavior
            if (blackboard->info.remain_hp >= 400){
                // according bullet to the buff
                if (blackboard->info.remain_bullet > 0){
                    if ( (!blackboard->info.has_buff && blackboard->info.times_to_buff >0 
                        && blackboard->GetDistance(blackboard->info.ally, blackboard->info.my_shield)>= blackboard->threshold.near_dist)
                        || blackboard->info.is_shielding)
                        {
                            cur_state = BehaviorStateEnum::SHIELD;
                      }
                    else if (!blackboard->info.has_my_enemy  && !blackboard->info.has_ally_enemy){
                        cur_state = BehaviorStateEnum::SEARCH;
                    }
                    else{
                        if (blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor ){
                            cur_state = BehaviorStateEnum::AMBUSH;
                    
                        }
                        else if(blackboard->info.has_ally_enemy){
                            cur_state = BehaviorStateEnum::ATTACK;
                        }
                    }
                }
                // not enough bullet
                else{
                    if (last_state == BehaviorStateEnum::SHIELD  && !blackboard->info.has_buff && blackboard->info.times_to_buff>0
                       && blackboard->GetDistance(mypose, blackboard->info.my_shield)<=blackboard->threshold.near_dist
                       || blackboard->info.is_shielding){
                         cur_state = BehaviorStateEnum::SHIELD;
                    }
                    else if ( ((blackboard->info.times_to_supply >0 
                       && (blackboard->GetDistance(blackboard->info.ally, blackboard->info.my_reload)>= blackboard->threshold.near_dist)) || blackboard->info.is_supplying
                       )
                       &&  !(blackboard->info.is_hitted && blackboard->info.remain_hp<=600)
                    ){
                        cur_state = BehaviorStateEnum::RELOAD;
                    }
                    else{
                        cur_state = BehaviorStateEnum::ESCAPE;
                    }

                }
                
            }
            // not enought hp
            else{
                if (blackboard->info.remain_bullet > 0){
                    if (!blackboard->info.has_my_enemy  && !blackboard->info.has_ally_enemy){
                        cur_state = BehaviorStateEnum::SEARCH;
                    }
                    else{
                        if (blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor ){
                            cur_state = BehaviorStateEnum::AMBUSH;
                    
                        }
                        else if(blackboard->info.has_ally_enemy){
                            cur_state = BehaviorStateEnum::ATTACK;
                        }
                    }
                
                }
                else if (blackboard->info.ally_remain_hp < blackboard->info.remain_hp && blackboard->info.times_to_supply >0  || blackboard->info.is_supplying){
                    cur_state = BehaviorStateEnum::RELOAD;
                }
                else{
                    cur_state = BehaviorStateEnum::ESCAPE;
                }
            }
            
    }

    // not first buff strategy--------------------------------------------------------------------------------------------------------
    else if (blackboard->info.strategy == "no_go_buff"){
            // state decision behavior
            if (blackboard->info.remain_hp >= 400){
                // according bullet to the buff
                if (blackboard->info.remain_bullet > 0){
                    if (!blackboard->info.has_my_enemy  && !blackboard->info.has_ally_enemy){
                        cur_state = BehaviorStateEnum::SEARCH;
                    }
                    else{
                        if (blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor ){
                            cur_state = BehaviorStateEnum::AMBUSH;
                    
                        }
                        else if(blackboard->info.has_ally_enemy){
                            cur_state = BehaviorStateEnum::ATTACK;
                        }
                    }
                }
                // not enough bullet
                else{
                    if ( ((blackboard->info.times_to_supply >0 
                       && (blackboard->GetDistance(blackboard->info.ally, blackboard->info.my_reload)>= blackboard->threshold.near_dist)) || blackboard->info.is_supplying
                       )
                       &&  !(blackboard->info.is_hitted && blackboard->info.remain_hp<=600))
                    {
                        cur_state = BehaviorStateEnum::RELOAD;
                    }
                    else if ( (!blackboard->info.has_buff && blackboard->info.times_to_buff >0 
                        && blackboard->GetDistance(blackboard->info.ally, blackboard->info.my_shield)>= blackboard->threshold.near_dist)
                        || blackboard->info.is_shielding){
                            cur_state = BehaviorStateEnum::SHIELD;
                    }
                    else{
                        cur_state = BehaviorStateEnum::ESCAPE;
                    }

                }
                
            }
            // not enought hp
            else{
                if (blackboard->info.remain_bullet > 0){
                    if (!blackboard->info.has_my_enemy  && !blackboard->info.has_ally_enemy){
                        cur_state = BehaviorStateEnum::SEARCH;
                    }
                    else{
                        if (blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor ){
                            cur_state = BehaviorStateEnum::AMBUSH;
                    
                        }
                        else if(blackboard->info.has_ally_enemy){
                            cur_state = BehaviorStateEnum::ATTACK;
                        }
                    }
                
                }
                else if (blackboard->info.ally_remain_hp <= blackboard->info.remain_hp && blackboard->info.times_to_supply >0  || blackboard->info.is_supplying){
                    cur_state = BehaviorStateEnum::RELOAD;
                }
                else{
                    cur_state = BehaviorStateEnum::ESCAPE;
                }
            }
            
    }


    // Defense together----------------------------------------------------------------------------------------------------------------
    else if (blackboard->info.strategy == "together"){
            // state decision behavior
            if (blackboard->info.remain_hp >= 400){
                // according bullet to the buff
                if (blackboard->info.remain_bullet > 0){
                    if ( (!blackboard->info.has_buff && blackboard->info.times_to_buff >0 
                        && blackboard->GetDistance(blackboard->info.ally, blackboard->info.my_shield)>= blackboard->threshold.near_dist)
                        || blackboard->info.is_shielding)
                        {
                            cur_state = BehaviorStateEnum::SHIELD;
                      }
                    else if (!blackboard->info.has_my_enemy  && !blackboard->info.has_ally_enemy && (blackboard->info.ally_remain_bullet>0 || blackboard->info.times_to_supply <=0)){
                        cur_state = BehaviorStateEnum::SEARCH;
                    }
                    // got enemy
                    else{
                        // fight together with ally has bullet.
                        if ((blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor) && (blackboard->info.ally_remain_bullet>0 || blackboard->info.times_to_supply<=0)){
                            cur_state = BehaviorStateEnum::AMBUSH;
                    
                        }
                        else if(blackboard->info.has_ally_enemy && (blackboard->info.ally_remain_bullet>0 || blackboard->info.times_to_supply<=0)){
                            cur_state = BehaviorStateEnum::ATTACK;
                        }
                        // fight with ally has not bullet, and I have bullet. and have times to bullet
                        else{
                            if ( ((blackboard->info.times_to_supply >0 
                                    && (blackboard->GetDistance(blackboard->info.ally, blackboard->info.my_reload)>= blackboard->threshold.near_dist)) || blackboard->info.is_supplying
                                    )
                                    &&  !(blackboard->info.is_hitted && blackboard->info.remain_hp<=600)){
                                cur_state = BehaviorStateEnum::RELOAD;
                            }
                            // finally Get out!
                            else{
                                cur_state = BehaviorStateEnum::ESCAPE;
                            }
                              
                        }

                    }
                }
                // not enough bullet
                else{
                    if (last_state == BehaviorStateEnum::SHIELD  && !blackboard->info.has_buff && blackboard->info.times_to_buff>0
                       && blackboard->GetDistance(mypose, blackboard->info.my_shield)<=blackboard->threshold.near_dist
                       || blackboard->info.is_shielding){
                         cur_state = BehaviorStateEnum::SHIELD;
                    }
                    else if ((blackboard->info.times_to_supply >0 
                       && (blackboard->GetDistance(blackboard->info.ally, blackboard->info.my_reload)>= blackboard->threshold.near_dist))
                    || blackboard->info.is_supplying){
                        cur_state = BehaviorStateEnum::RELOAD;
                    }
                    else{
                        cur_state = BehaviorStateEnum::ESCAPE;
                    }

                }
                
            }
            // not enought hp
            else{
                if (blackboard->info.remain_bullet > 0){
                    if (!blackboard->info.has_my_enemy  && !blackboard->info.has_ally_enemy){
                        cur_state = BehaviorStateEnum::SEARCH;
                    }
                    else{
                        if (blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor ){
                            cur_state = BehaviorStateEnum::AMBUSH;
                    
                        }
                        else if(blackboard->info.has_ally_enemy){
                            cur_state = BehaviorStateEnum::ATTACK;
                        }
                    }
                
                }
                else if (blackboard->info.ally_remain_hp <= blackboard->info.remain_hp && blackboard->info.times_to_supply >0  || blackboard->info.is_supplying){
                    cur_state = BehaviorStateEnum::RELOAD;
                }
                else{
                    cur_state = BehaviorStateEnum::ESCAPE;
                }
            }
            
    }
    
    // attack originally for chasing
    else if (blackboard->info.strategy == "attack"){
         // state decision behavior
            if (blackboard->info.remain_hp >= 400){
                if (blackboard->info.remain_bullet > 0){
                    //   if (!blackboard->info.has_buff){
                    //         cur_state = BehaviorStateEnum::SHIELD;
                    //   }
                    //   else 
                    if (!blackboard->info.has_my_enemy  && !blackboard->info.has_ally_enemy && !blackboard->info.is_chase){
                        cur_state = BehaviorStateEnum::SEARCH;
                    }
                    else{
                        if (blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor || blackboard->info.is_chase ){
                            cur_state = BehaviorStateEnum::CHASE;
                    
                        }
                        else if(blackboard->info.has_ally_enemy){
                            cur_state = BehaviorStateEnum::ATTACK;
                        }
                        //    cur_state = BehaviorStateEnum::CHASE;
                        //    cur_state = BehaviorStateEnum::AMBUSH;
                        //   cur_state = BehaviorStateEnum::ATTACK;
                    }
                }
                else if (blackboard->info.times_to_supply >0){
                    cur_state = BehaviorStateEnum::RELOAD;  
                }
                else{
                    cur_state = BehaviorStateEnum::ESCAPE;  
                }
            }
            else{
                if (blackboard->info.remain_bullet > 0){
                    if (!blackboard->info.has_my_enemy  && !blackboard->info.has_ally_enemy){
                        cur_state = BehaviorStateEnum::PATROL;
                    }
                    else{
                        cur_state = BehaviorStateEnum::CHASE;
                    }
                
            }
            else{
                cur_state = BehaviorStateEnum::BACKBOOT;
            }
            }
    }

    // Defense
    else if (blackboard->info.strategy == "defense"){
            // state decision behavior
            if (blackboard->info.remain_hp >= 400){
                if (blackboard->info.remain_bullet > 0){
                    //   if (!blackboard->info.has_buff){
                    //         cur_state = BehaviorStateEnum::SHIELD;
                    //   }
                    //   else 
                    if (!blackboard->info.has_my_enemy  && !blackboard->info.has_ally_enemy){
                        cur_state = BehaviorStateEnum::SEARCH;
                    }
                    else{
                        if (blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor ){
                            cur_state = BehaviorStateEnum::AMBUSH;
                    
                        }
                        else if(blackboard->info.has_ally_enemy){
                            cur_state = BehaviorStateEnum::ATTACK;
                        }
                        //    cur_state = BehaviorStateEnum::CHASE;
                        //    cur_state = BehaviorStateEnum::AMBUSH;
                        //   cur_state = BehaviorStateEnum::ATTACK;
                    }
                }
                else if (blackboard->info.times_to_supply >0){
                    cur_state = BehaviorStateEnum::RELOAD;  
                }
                else{
                    cur_state = BehaviorStateEnum::ESCAPE;  
                }
            }
            else{
                if (blackboard->info.remain_bullet > 0){
                    if (!blackboard->info.has_my_enemy  && !blackboard->info.has_ally_enemy){
                        cur_state = BehaviorStateEnum::PATROL;
                    }
                    else{
                        cur_state = BehaviorStateEnum::AMBUSH;
                    }
                
                }
                else if (blackboard->info.ally_remain_hp <400 && blackboard->info.times_to_supply >0){
                    cur_state = BehaviorStateEnum::RELOAD;
                }
                else{
                    cur_state = BehaviorStateEnum::SHIELD;
                }
            }
            
    }
      
    
    // filter
    if (!(last_state == BehaviorStateEnum::RELOAD  ||  last_state == BehaviorStateEnum::SHIELD)){
        count = last_state != cur_state ? count + 1: 0;
        cur_state = count>=count_bound? cur_state: last_state;
    }
    
   
   // cancel last state
    if ( (last_state != BehaviorStateEnum::INIT && last_state != cur_state)  || blackboard->info.remain_hp<=0 ){
         switch (last_state){
            case BehaviorStateEnum::BACKBOOT:
                back_boot_area_behavior.Cancel();
                break;
            case BehaviorStateEnum::CHASE:
                chase_behavior.Cancel();
                break;
            case BehaviorStateEnum::ESCAPE:
                escape_behavior.Cancel();
                break;
            case BehaviorStateEnum::PATROL:
                patrol_behavior.Cancel();
                break;
            case BehaviorStateEnum::RELOAD:
                reload_behavior.Cancel();
                break;
            case BehaviorStateEnum::SHIELD:
                shield_behavior.Cancel();
                break;
            case BehaviorStateEnum::SEARCH:
                search_behavior.Cancel();
                break;
            case BehaviorStateEnum::AMBUSH:
                ambush_behavior.Cancel();
                break;
            case BehaviorStateEnum::ATTACK:
                attack_behavior.Cancel();
                break;
         }
    }

    // remain hp is 0, them dead!
    if (blackboard->info.remain_hp <=0){
        ros::shutdown();
        break;
    }

    switch (cur_state){
      case BehaviorStateEnum::BACKBOOT:
          back_boot_area_behavior.Run();
          std::cout<<"BackBoot" << std::endl;
          break;
      case BehaviorStateEnum::CHASE:
          chase_behavior.Run();
          std::cout<<"CHASE" << std::endl;
          break;
      case BehaviorStateEnum::ESCAPE:
          escape_behavior.Run();
          std::cout<<"ESCAPE" << std::endl;
          break;
      case BehaviorStateEnum::PATROL:
          patrol_behavior.Run();
          std::cout<<"PATROL" << std::endl;
          break;
      case BehaviorStateEnum::RELOAD:
          reload_behavior.Run();
          std::cout<<"RELOAD" << std::endl;
          break;
      case BehaviorStateEnum::SHIELD:
          shield_behavior.Run();
          std::cout<<"SHIELD" << std::endl;
          break;
      case BehaviorStateEnum::SEARCH:
          search_behavior.Run();
          std::cout<<"SEARCH" << std::endl;
          break;
      case BehaviorStateEnum::AMBUSH:
          ambush_behavior.Run();
          std::cout<<"AMBUSH" << std::endl;
          break;
      case BehaviorStateEnum::ATTACK:
          attack_behavior.Run();
          std::cout<<"ATTACK" << std::endl;
          break;


    }

    last_state = cur_state;
    
    rate.sleep(); 
 
   
    
  }


  return 0;
}

