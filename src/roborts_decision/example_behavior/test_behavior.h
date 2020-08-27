#ifndef ROBORTS_DECISION_TEST_BEHAVIOR_H
#define ROBORTS_DECISION_TEST_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class TestBehavior {
 public:
    TestBehavior(ChassisExecutor* &chassis_executor,
                  Blackboard* &blackboard,
                  const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                         blackboard_(blackboard) {

     

  }

  void Run() {
    auto executor_state = Update();
    
    if (blackboard_->info.remain_bullet >0 && blackboard_->info.heat <110){
         blackboard_->Shoot(blackboard_->info.shoot_hz);
    }
    else{
         blackboard_->StopShoot();
    }
    
    
    // // ambush points
    // if (
    //   blackboard_->info.has_my_enemy
    //   // && blackboard_->info.found_enemy   ? found_enemy always not valid.
    //   ){
    //     blackboard_->Shoot(1);
    // }
    // else{
    //     blackboard_->StopShoot();
    // }
    
    
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  

  ~TestBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

 

};
}

#endif //ROBORTS_DECISION_TEST_BEHAVIOR_H
