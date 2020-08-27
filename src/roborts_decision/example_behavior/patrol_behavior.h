#ifndef ROBORTS_DECISION_PATROL_BEHAVIOR_H
#define ROBORTS_DECISION_PATROL_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class PatrolBehavior {
 public:
  PatrolBehavior(ChassisExecutor* &chassis_executor,
                 Blackboard* &blackboard,
                 const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                        blackboard_(blackboard)
                                                         {

    patrol_count_ = 0;
    point_size_ = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {
    static int count = 0;
    auto executor_state = Update();

    std::cout << "state: " << (int)(executor_state) << std::endl;

    if (executor_state != BehaviorState::RUNNING) {
      geometry_msgs::PoseStamped goal;
      if (patrol_goals_.empty()) {
        ROS_ERROR("patrol goal is empty");
        return;
      }

      

      // count = (count + 1)%2;
      // if (count==0){
      //   goal = blackboard_->info.my_reload;
      // }
      // else{
      //   goal = blackboard_->info.my_shield;
      // }
      // auto goal = patrol_goals_[patrol_count_];
      // patrol_count_ = ++patrol_count_ % point_size_;
      
      
      
      
      // random select
      // reload shield or patrol
      std::uniform_int_distribution<int> sel(0, 1);
      if (sel(generator)==0){
         goal = blackboard_->info.my_shield;
         
      }
      else{
         std::uniform_int_distribution<int> dis(0, point_size_-1); 
         int sel_i = dis(generator);
         goal = patrol_goals_[sel_i];
      }
      
      
      if (!blackboard_->IsBombAllyGoal(goal)){
         chassis_executor_->Execute(goal);
         blackboard_->SetMyGoal(goal);
      }

    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    point_size_ = (unsigned int)decision_config.blue().patrol().size();
    patrol_goals_.resize(point_size_);
    for (int i = 0; i != point_size_; i++) {
      if (blackboard_->info.team_blue){
         patrol_goals_[i] = blackboard_->Point2PoseStamped(decision_config.blue().patrol(i));
      }
      else{
         patrol_goals_[i] = blackboard_->Point2PoseStamped(decision_config.red().patrol(i));
      }
      
    }

    return true;
  }

  ~PatrolBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! patrol buffer
  std::vector<geometry_msgs::PoseStamped> patrol_goals_;
  unsigned int patrol_count_;
  unsigned int point_size_;
  std::minstd_rand   generator;  


  

};
}

#endif //ROBORTS_DECISION_PATROL_BEHAVIOR_H
