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

void Command();
char command = 'a';

int main(int argc, char **argv) {
  ros::init(argc, argv, "sel_behavior_node");
  
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/sel_behave.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);

  
  // Behaviors
  roborts_decision::BackBootAreaBehavior back_boot_area_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::ChaseBehavior        chase_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::SearchBehavior       search_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::EscapeBehavior       escape_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::PatrolBehavior       patrol_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::GoalBehavior       goal_behavior(chassis_executor, blackboard);
  roborts_decision::ReloadBehavior     reload_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::ShieldBehavior     shield_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::TestBehavior      test_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::AttackBehavior    attack_behavior(chassis_executor, blackboard, full_path);


  auto command_thread= std::thread(Command);
  ros::Rate rate(10);

  


  while(ros::ok()){
    ros::spinOnce();
    // if (blackboard->CanDodge()){
    //     blackboard->StartDodge();
    // }
    
   if (command !='a'){
    // printf("Can Dodge:%d\n", blackboard->CanDodge());

    printf("-----------------------------------------\n");
    printf("Remaining Time:%d and is begin:%d\n", blackboard->info.remaining_time, blackboard->info.is_begin);
    printf("Times to supply:%d\n", blackboard->info.times_to_supply);
    // printf("Is Stuck:%d, Can Dodge:%d\n", blackboard->IsInStuckArea(), blackboard->CanDodge());
    printf("Ally: \n");
    printf("hp:%d, bullet:%d,  pose:(%f, %f)\n", blackboard->info.ally_remain_hp, blackboard->info.ally_remain_bullet, blackboard->info.ally.pose.position.x, blackboard->info.ally.pose.position.y);
    printf("has_ally_enemy:%d, has_ally_first_enemy:%d, has_ally_second_endmy:%d\n", blackboard->info.has_ally_enemy, blackboard->info.has_ally_first_enemy, blackboard->info.has_ally_second_enemy);
    printf("ally first enemy pose:(%f, %f), ally second enemy pose:(%f, %f)\n", blackboard->info.ally_first_enemy.pose.position.x, blackboard->info.ally_first_enemy.pose.position.y, blackboard->info.ally_second_enemy.pose.position.x, blackboard->info.ally_second_enemy.pose.position.y);
    printf("\n");
    printf("My:\n");
    printf("hp:%d, bullet:%d,  pose:(%f, %f)\n", blackboard->info.remain_hp, blackboard->info.remain_bullet, blackboard->GetRobotMapPose().pose.position.x, blackboard->GetRobotMapPose().pose.position.y);    
    printf("has_my_enemy:%d, has_first_enemy:%d, has_second_enemy:%d\n", blackboard->info.has_my_enemy, blackboard->info.has_first_enemy, blackboard->info.has_second_enemy);  
    printf("my first enemy pose:(%f, %f), my second enemy pose:(%f, %f)\n", blackboard->info.first_enemy.pose.position.x, blackboard->info.first_enemy.pose.position.y, blackboard->info.second_enemy.pose.position.x, blackboard->info.second_enemy.pose.position.y);
    printf("my goal:(%f, %f), ally goal:(%f, %f)\n", blackboard->info.my_goal.pose.position.x, blackboard->info.my_goal.pose.position.y, blackboard->info.ally_goal.pose.position.x, blackboard->info.ally_goal.pose.position.y);
    

   }
    switch (command) {
      // test behavior
      case '0':
        test_behavior.Run();
        printf("State: TEST\n");
        break;
      //back to boot area
      case '1':
        back_boot_area_behavior.Run();
        printf("State: BackBoot\n");
        break;
        //patrol
      case '2':
        patrol_behavior.Run();
        printf("State: Patrol\n");
        break;
        //chase.
      case '3':
        chase_behavior.Run();
        printf("State: Chase\n");
        break;
        //search
      case '4':
        search_behavior.Run();
        printf("State: Search\n");
        break;
        //escape.
      case '5':
        escape_behavior.Run();
        printf("State: Escape\n");
        break;
        //goal.
      case '6':
        goal_behavior.Run();
        printf("State: Goal\n");
        break;
        // reload
      case '7':
        reload_behavior.Run();
        printf("State: Reload\n");
        break;
      case '8':
        shield_behavior.Run();
        printf("State: Shield\n");
        break;

      case '9':
        attack_behavior.Run();
        printf("State: Attack\n");
        break;
        
      case 27:
        if (command_thread.joinable()){
          command_thread.join();
        }
        return 0;
      default:
        break;
    }
    rate.sleep();
  }


  return 0;
}

void Command() {

  while (command != 27) {
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "*********************************please send a command********************************" << std::endl;   
    std::cout << "0: test behavior" << std::endl
              << "1: back boot area behavior" << std::endl
              << "2: patrol behavior" << std::endl
              << "3: chase_behavior" << std::endl
              << "4: search behavior" << std::endl
              << "5: escape behavior" << std::endl
              << "6: goal behavior" << std::endl
              << "7: reload behavior" << std::endl
              << "8: shield behavior" << std::endl
              << "9: attack behavior" << std::endl
              << "esc: exit program" << std::endl;
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "> ";
    std::cin >> command;
    // if (command != '0' &&  command != '1' && command != '2' && command != '3' && command != '4' && command != '5' && command != '6' && 
    //     && command != "7" && command != "8" && command != "9" && command != 27) {
    //   std::cout << "please input again!" << std::endl;
    //   std::cout << "> ";
    //   std::cin >> command;
    // }

  }
}

