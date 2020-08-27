#ifndef ROBORTS_DECISION_NN_BEHAVIOR_H
#define ROBORTS_DECISION_NN_BEHAVIOR_H



#include <iostream>
#include <memory>

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"
#include <torch/script.h> // One-stop header.


namespace roborts_decision {
class NNBehavior {
 public:
  NNBehavior(ChassisExecutor* &chassis_executor,
                 Blackboard* &blackboard,
                 const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                        blackboard_(blackboard)
                                                         {

 
    point_size_ = 0;
    last_act = -1;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

    // Deserialize the ScriptModule from a file using torch::jit::load().
    module = torch::jit::load("/home/drl/RM/weights/agent_model.pt");
    assert(module != nullptr);
    std::cout << "Load Model OK!\n";
    hidden_state = torch::zeros({1, 64});


  }

  void Run() {

    auto executor_state = Update();
    if (patrol_goals_.empty()) {
          ROS_ERROR("patrol goal is empty");
          return;
    }

    if (executor_state != BehaviorState::RUNNING ) {
        std::vector<float>  obs = get_obs();
        assert(obs.size() == 71);
        float obs_[71];
        
        for (int i=0; i<71; i++){
            obs_[i] = obs[i];
        }
      
        torch::jit::IValue input_state = torch::from_blob(obs_,{1, 71});
        torch::jit::IValue output = module->forward({input_state, hidden_state});

        auto a =  output.toTuple();
        torch::Tensor actions = a->elements().at(0).toTensor();
        hidden_state = a->elements().at(1);

        int action_size = actions.size(1);
        std::vector<int> arg_i;

        for (int i=0; i<action_size; i++)
           arg_i.push_back(actions.argsort(-1, true)[0][i].item().to<int>());
       
        bool ok = false;
        int count=0;
        static int last_sel_i =-1;
        int sel_i;
        geometry_msgs::PoseStamped goal;
        while (!ok && count<action_size){
            sel_i = arg_i[count];
            if (sel_i<4){
                    switch (sel_i){
                    case 0:  {goal = blackboard_->info.start_position; break;}
                    case 1:  {goal = blackboard_->GetRobotMapPose(); break;}
                    case 2:  {goal = blackboard_->info.my_reload; break;}
                    case 3:  {goal = blackboard_->info.my_shield; break;}
                }
            }
            else if (sel_i>=4 && sel_i<=13){
                goal = patrol_goals_[sel_i-4];
            }
            else{
                goal = blackboard_->GetEnemy();
            }

            if (
                // !blackboard_->IsBombAllyGoal(goal) 
                goal.pose.position.x!=-99 && goal.pose.position.y !=-99
                && last_act != sel_i ){

                    chassis_executor_->Execute(goal);
                    blackboard_->SetMyGoal(goal);
                    last_act = sel_i;
                    ok = true;
                    printf("NN Sel:%d, Master:%d\n", sel_i, blackboard_->info.is_master);
                    printf("Goal Sel:(%f, %f)\n", goal.pose.position.x, goal.pose.position.y);
            }
      

            count++;
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

    point_size_ = (unsigned int)(decision_config.blue().patrol().size());
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

  ~NNBehavior() = default;

 private:


 std::vector<float> get_obs(){
      int d_x[8] = { 0,   1,   1,   1,   0,   -1,   -1,   -1};
      int d_y[8] = {-1, -1,   0,   1,   1,    1,    0,   -1};
      float  avail_move_feat[12];   // move: 12 
      float  obs_dist_feat[8] ;     // obs dist: 8 
      float  enemy_feat[2*4];         // 2 * (avail: 1, dist: 1, relative_x: 1, relative_y: 1)
      float  ally_feat[7];     // visible: 1; dist: 1; relative_x_y: 2;  , health:1, buff:1, bullet:1,  
      float  last_act_feat[16];  // last_action 
      float  own_feat[18]; // hp, buff, bullet:3, pos relative x_y:10, dist to 5 points: 5
      float  agent_id[2];

      std::vector<float>  total_feat;
      double norm = 100;
      
      // initialize avail_move_feat-----------------------------------------------------------------------------
      if (blackboard_->info.remain_hp>0){
          // my reload
          avail_move_feat[0] = canMove(blackboard_->info.my_reload);
          // my shield
          avail_move_feat[1] = canMove(blackboard_->info.my_shield);
          // patrol_pos
          for (int i=0; i<patrol_goals_.size(); i++)
            avail_move_feat[2 + i] = canMove(patrol_goals_[i]);
          
      }
      else // if dead
      {
          for (int i=0; i<12; i++)  avail_move_feat[i] = 0;
      }

      // initialize obs dist feat------------------------------------------------------------------------------------
      geometry_msgs::PoseStamped my_robot = blackboard_->GetRobotMapPose();
      // down ,down right, right, up right, up, up left, left, down left
      auto cost_ptr = blackboard_->GetCostMap2D();
      double obs_x, obs_y, dist;
      unsigned int s_x, s_y;
      int up_bound_x, up_bound_y, down_bound_x, down_bound_y;

      up_bound_x = cost_ptr->GetSizeXCell();
      up_bound_y = cost_ptr->GetSizeYCell();
      down_bound_x = cost_ptr->GetOriginX();
      down_bound_y = cost_ptr->GetOriginY();

      cost_ptr->World2Map(my_robot.pose.position.x, my_robot.pose.position.y, s_x, s_y);
      for (int i=0; i<8; i++){
           int e_x, e_y;
           e_x = s_x; 
           e_y = s_y;
           while (e_x>down_bound_x && e_y>down_bound_y && e_x<up_bound_x-1 && e_y < up_bound_y - 1 ){
               if (cost_ptr->GetCost(e_x, e_y)>=253)
                  break;
               e_x = e_x + d_x[i];
               e_y = e_y + d_y[i];
           }
           cost_ptr->Map2World(e_x, e_y, obs_x, obs_y);
           obs_dist_feat[i] = blackboard_->GetEulerDistance(obs_x, obs_y, my_robot.pose.position.x ,my_robot.pose.position.y) / norm;
      }

      // initialize enemy_feat-------------------------------------------------------------------------------------
      for (int i=0; i<2*4; i++)  enemy_feat[i] = 0.0;
      if (blackboard_->info.has_first_enemy){
          enemy_feat[0] = 1;
          enemy_feat[1] = (blackboard_->info.first_enemy.pose.position.x - my_robot.pose.position.x) / norm;
          enemy_feat[2] = (blackboard_->info.first_enemy.pose.position.y - my_robot.pose.position.y) / norm;
          enemy_feat[3] = blackboard_->GetDistance(blackboard_->info.first_enemy, my_robot) / norm;
      }
      if (blackboard_->info.has_second_enemy){
          enemy_feat[4] = 1;
          enemy_feat[5] = (blackboard_->info.second_enemy.pose.position.x - my_robot.pose.position.x) / norm;
          enemy_feat[6] = (blackboard_->info.second_enemy.pose.position.y - my_robot.pose.position.y) / norm;
          enemy_feat[7] = blackboard_->GetDistance(blackboard_->info.second_enemy, my_robot) / norm;
      }

      // initialize ally_feat-------------------------------------------------------------------------------------
      if (blackboard_->info.has_ally){
          ally_feat[0] = 1;
          ally_feat[1] = blackboard_->GetDistance(my_robot, blackboard_->info.ally) / norm;
          ally_feat[2] = (blackboard_->info.ally.pose.position.x - my_robot.pose.position.x) / norm;
          ally_feat[3] = (blackboard_->info.ally.pose.position.y - my_robot.pose.position.y) / norm;
          ally_feat[4] = blackboard_->info.ally_remain_hp / 2000;
          ally_feat[5] = int(blackboard_->info.has_buff);
          ally_feat[6] = blackboard_->info.ally_remain_bullet / 200;
      }
      else{
          for (int i=0; i<7; i++) ally_feat[i] = 0.0;
      }
      

      // initialize last_act_feat--------------------------------------------------------------------------------
      for (int i=0; i<16; i++)  last_act_feat[i] = 0.0;
      if (last_act != -1)  last_act_feat[last_act] = 1.0;

      // initialize own_feat-------------------------------------------------------------------------------------  
      own_feat[0] = blackboard_->info.remain_hp / 2000;
      own_feat[1] = int(blackboard_->info.has_buff);
      own_feat[2] = blackboard_->info.remain_bullet / 200;
      // relative pos x pos y
      own_feat[3] = (my_robot.pose.position.x - 4.0) / norm;  // center
      own_feat[4] = (my_robot.pose.position.y - 2.5) / norm;  // center
      own_feat[5] = (my_robot.pose.position.x - blackboard_->info.my_reload.pose.position.x) / norm;  // my reload
      own_feat[6] = (my_robot.pose.position.y - blackboard_->info.my_reload.pose.position.y) / norm;  // my reload
      own_feat[7] = (my_robot.pose.position.x - blackboard_->info.my_shield.pose.position.x) / norm;  // my shield
      own_feat[8] = (my_robot.pose.position.y - blackboard_->info.my_shield.pose.position.y) / norm;  // my shield
      own_feat[9] = (my_robot.pose.position.x - blackboard_->info.opp_reload.pose.position.x) / norm;  // opp reload
      own_feat[10] = (my_robot.pose.position.y - blackboard_->info.opp_reload.pose.position.y) / norm;  // opp reload
      own_feat[11] = (my_robot.pose.position.x - blackboard_->info.opp_shield.pose.position.x) / norm;  // opp shield
      own_feat[12] = (my_robot.pose.position.y - blackboard_->info.opp_shield.pose.position.y) / norm;  // opp shield
      // dist 
      own_feat[13] = blackboard_->GetEulerDistance(my_robot.pose.position.x, my_robot.pose.position.y, 4.0, 2.5) / norm;
      own_feat[14] = blackboard_->GetDistance(my_robot, blackboard_->info.my_reload) / norm;
      own_feat[15] = blackboard_->GetDistance(my_robot, blackboard_->info.my_shield) / norm;
      own_feat[16] = blackboard_->GetDistance(my_robot, blackboard_->info.opp_reload) / norm;
      own_feat[17] = blackboard_->GetDistance(my_robot, blackboard_->info.opp_shield) / norm;

      if (blackboard_->info.is_master){
          agent_id[0] = 1; agent_id[1] = 0;
      }
      else{
          agent_id[0] = 0; agent_id[1] = 1;
      }

      // load feature---------------------------------------------------------------------------------
      // avail
      for (int i=0; i<12; i++){
          total_feat.push_back(avail_move_feat[i]);
      }
      // obs dist
      for (int i=0; i<8; i++){
          total_feat.push_back(obs_dist_feat[i]);
      }
      // enemy_feat
      for (int i=0; i<8; i++){
          total_feat.push_back(enemy_feat[i]);
      }
      // ally_feat
      for (int i=0; i<7; i++){
          total_feat.push_back(ally_feat[i]);
      }
    
      // own_feat
      for (int i=0; i<18; i++){
          total_feat.push_back(own_feat[i]);
      }

      // last_act_feat
      for (int i=0; i<16; i++){
          total_feat.push_back(last_act_feat[i]);
      }
      
      // agent_id
      for (int i=0; i<2; i++){
          total_feat.push_back(agent_id[i]);
      }

      return total_feat;


  }

  bool nearThreshold(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2){
      double dist = blackboard_->GetDistance(p1, p2);
      return dist <= 0.5;
  }

  float canMove(geometry_msgs::PoseStamped goal){
      // ally
      geometry_msgs::PoseStamped detect;
      detect = blackboard_->info.ally;
      if (nearThreshold(goal, detect))
         return 0.0;
      // enemy
      if (blackboard_->info.has_first_enemy){
        detect = blackboard_->info.first_enemy;
        if (nearThreshold(goal, detect))
           return 0.0;
      }
      if (blackboard_->info.has_second_enemy){
        detect = blackboard_->info.second_enemy;
        if (nearThreshold(goal, detect))
          return 0.0;
      }

      if (blackboard_->info.has_ally_first_enemy){
        detect = blackboard_->info.ally_first_enemy;
        if (nearThreshold(goal, detect))
          return 0.0;
      }

      if (blackboard_->info.has_ally_second_enemy){
        detect = blackboard_->info.ally_second_enemy;
        if (nearThreshold(goal, detect))
          return 0.0;
      }

      return 1.0;
  }


  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! patrol buffer
  std::vector<geometry_msgs::PoseStamped> patrol_goals_;
  unsigned int point_size_;
  std::minstd_rand   generator;  

  std::shared_ptr<torch::jit::script::Module> module;
  torch::jit::IValue hidden_state;
  float obs[71];

  int last_act;



  

};
}

#endif //ROBORTS_DECISION_NN_BEHAVIOR_H
