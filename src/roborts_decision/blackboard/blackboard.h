/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "roborts_msgs/ArmorDetectionAction.h"


// roborts_msgs 
#include "roborts_msgs/ArmorPos.h"
#include "roborts_msgs/ArmorsPos.h"
#include "roborts_msgs/ShooterCmd.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/RobotHeat.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/RobotShoot.h"
#include "roborts_msgs/ProjectileSupply.h"
#include "roborts_msgs/SupplierStatus.h"
#include "roborts_msgs/BonusStatus.h"

// #include "roborts_msgs/AllyPose.h"
#include "roborts_msgs/FusionTarget.h"
#include "roborts_msgs/Target.h"
#include "roborts_msgs/DodgeMode.h"
#include "roborts_msgs/GimbalAngle.h"




#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"

// communication.h
#include "communication.h"


// wifi part
#include <zmq.hpp>
#include <unistd.h>
#include <thread>



namespace roborts_decision{


struct Threshold{
    float near_dist;
    float near_angle;
    float heat_upper_bound;
    float detect_dist;

};

struct DecisionInfoPool{
  int remaining_time;
  int times_to_supply;
  int times_to_buff;
  int game_status;
  int shoot_hz;
  bool is_begin;
  bool is_master;
  bool team_blue;
  bool has_buff;
  bool has_ally;
  bool has_my_enemy;
  bool has_ally_enemy;
  bool has_first_enemy;
  bool has_second_enemy;
  bool can_shoot;
  bool can_dodge;
  bool is_supplying;
  bool is_shielding;
  bool got_last_enemy;
  bool use_refree;

  // ally enemy part
  bool has_ally_first_enemy;
  bool has_ally_second_enemy;

  bool is_hitted;
  bool is_chase;
  bool valid_camera_armor;
  bool valid_front_camera_armor;
  bool valid_back_camera_armor;
  int remain_bullet;
  int remain_hp;
  int frequency;
  float speed;
  int ally_remain_bullet;
  int ally_remain_hp;
  int heat;
  double ally_dist;
  double first_enemy_dist;
  double second_enemy_dist;
  float ally_yaw;
  std::string strategy;
  geometry_msgs::PoseStamped ally;
  geometry_msgs::PoseStamped first_enemy;
  geometry_msgs::PoseStamped second_enemy;
  // ally enemy part
  geometry_msgs::PoseStamped ally_first_enemy;
  geometry_msgs::PoseStamped ally_second_enemy;
  // all goal
  geometry_msgs::PoseStamped my_goal;
  geometry_msgs::PoseStamped ally_goal;

  geometry_msgs::PoseStamped last_enemy;

  geometry_msgs::PoseStamped my_reload;
  geometry_msgs::PoseStamped my_shield;
  geometry_msgs::PoseStamped opp_reload;
  geometry_msgs::PoseStamped opp_shield;
  geometry_msgs::PoseStamped start_position;
};


class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  explicit Blackboard(const std::string &proto_file_path):
      enemy_detected_(false),
      is_in_shoot_state(false),
      armor_detection_actionlib_client_("armor_detection_node_action", true),
      context_(1), masterSocket_(context_, ZMQ_REP), clientSocket_(context_, ZMQ_REQ){

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    std::string map_path = ros::package::getPath("roborts_costmap") + \
      "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();
   
    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

    // Enemy fake pose
    ros::NodeHandle rviz_nh("/move_base_simple");
    enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::GoalCallback, this);

    // node handle
    ros::NodeHandle nh;
    armor_sub_ = nh.subscribe<roborts_msgs::ArmorsPos>("enemies", 10, &Blackboard::ArmorCallback, this);
    camera_armor_sub_ = nh.subscribe<roborts_msgs::ArmorsPos>("front_camera_robot_pos", 10, &Blackboard::FrontCameraArmorCallback, this);  // Front Camera
    back_camera_sub_ = nh.subscribe<roborts_msgs::ArmorsPos>("back_camera_robot_pos", 10, &Blackboard::BackCameraArmorCallback, this);   // Back Camera        
    robot_status_sub_ = nh.subscribe<roborts_msgs::RobotStatus>("robot_status", 10, &Blackboard::RobotStatusCallback, this);
    robot_heat_sub_ = nh.subscribe<roborts_msgs::RobotHeat>("robot_heat", 10, &Blackboard::RobotHeatCallback, this);
    robot_shoot_sub_ = nh.subscribe<roborts_msgs::RobotShoot>("robot_shoot", 10, &Blackboard::RobotShootCallback, this);
    game_status_sub_ = nh.subscribe<roborts_msgs::GameStatus>("game_status", 10, &Blackboard::GameStatusCallback, this);
    fusion_target_sub_ = nh.subscribe<roborts_msgs::FusionTarget>("laser_camera_fusion", 10, &Blackboard::FusionDetectionCallback, this);
    supply_sub_ = nh.subscribe<roborts_msgs::SupplierStatus>("field_supplier_status", 10, &Blackboard::FieldSupplyCallback,this);
    cmd_gimbal_sub_ = nh.subscribe<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 10, &Blackboard::CmdGimbalCallback, this);
    vel_acc_sub_ = nh.subscribe<roborts_msgs::TwistAccel>("cmd_vel_acc", 10, &Blackboard::VelAccCallback, this);
    buff_sub_ = nh.subscribe<roborts_msgs::BonusStatus>("field_bonus_status", 10, &Blackboard::BuffCallback, this);

    // pub
    shoot_pub_ = nh.advertise<roborts_msgs::ShooterCmd>("shoot_cmd", 10, this);
    ally_pub_ = nh.advertise<geometry_msgs::PoseStamped>("friend_pose", 10, this);
    fusion_pub_ = nh.advertise<roborts_msgs::FusionTarget>("communication_robots",10, this);
    dodge_pub_ = nh.advertise<roborts_msgs::DodgeMode>("dodge_mode", 10, this);
    supply_pub_ = nh.advertise<roborts_msgs::ProjectileSupply>("projectile_supply", 10, this);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10, this);



    // all frame ID update
    std::string tf_prefix = tf::getPrefixParam(nh);
    pose_frameID = "base_link";
    gimbal_frameID = "gimbal";
    if (!tf_prefix.empty()) {
        pose_frameID = tf::resolve(tf_prefix, pose_frameID);
        gimbal_frameID = tf::resolve(tf_prefix, gimbal_frameID);
    }
    

    roborts_decision::DecisionConfig decision_config;
    roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);
    use_camera = decision_config.use_camera();
    if (use_camera){

      armor_detection_actionlib_client_.waitForServer();

      ROS_INFO("Armor detection module has been connected!");

      armor_detection_goal_.command = 1;
      armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
                                                 boost::bind(&Blackboard::ArmorDetectionFeedbackCallback, this, _1));
    }


    // initialize setting----------------------------------------------------------
    info.remaining_time = 299;
    info.times_to_supply = 2;
    info.times_to_buff = 1;
    info.shoot_hz = decision_config.shoot_hz();
    info.use_refree = decision_config.use_refree();
    info.is_begin = false;
    info.is_master = decision_config.master();
    info.team_blue = decision_config.isblue();
    info.can_shoot = decision_config.can_shoot();
    info.can_dodge = decision_config.can_dodge();
    info.has_buff = false;
    info.has_ally = false;
    info.has_my_enemy = false;
    info.has_ally_enemy = false;
    info.has_first_enemy = false;
    info.has_second_enemy = false;
    info.has_ally_first_enemy = false;
    info.has_ally_second_enemy = false;
    info.is_hitted = false;
    info.is_chase = false;
    info.is_supplying = false;
    info.is_shielding = false;
    info.got_last_enemy = false;

    info.valid_camera_armor = false;
    info.valid_back_camera_armor = false;
    info.valid_front_camera_armor = false;
    info.remain_bullet = decision_config.remain_bullet();
    info.remain_hp = 2000;
    remain_hp = info.remain_hp;
    info.ally_remain_bullet = 50;
    info.ally_remain_hp = 2000;
    info.first_enemy_dist = 0;
    info.second_enemy_dist = 0;
    info.ally_dist = 0;
    info.heat = 0;
    info.frequency = 0;
    info.speed = 0.0;
    info.strategy = decision_config.strategy();

    // goal passport
    info.my_goal = InitMapPose();
    info.ally_goal = InitMapPose();
    

    // PostStamped information update
    info.ally = InitMapPose();
    info.ally_first_enemy = InitMapPose();
    info.ally_second_enemy = InitMapPose();
    info.last_enemy = InitMapPose();
   

    // initalize threshold----------------------------------------------------------------------
    threshold.near_dist = 0.5;  // m
    threshold.near_angle =0.2;  // rad ~ 10 degree
    threshold.detect_dist = 5.0;  // m
    threshold.heat_upper_bound = 110; // limit

    // initialize enemy
    in_dodge = false;
    _front_first_enemy = false;
    _front_second_enemy = false;
    _back_first_enemy = false;
    _back_second_enemy = false;
    my_shoot_1_cnt = 0;
    my_shoot_2_cnt = 0;
    ally_shoot_1_cnt = 0;
    ally_shoot_2_cnt = 0;

    // dodge config
    _dodge_in_reload = decision_config.dodge_in_reload();

    

    // load all shield reload points
    if (info.team_blue){
        info.my_reload = Point2PoseStamped(decision_config.blue().reload_point());
        info.my_shield = Point2PoseStamped(decision_config.blue().shield_point());
        info.opp_reload = Point2PoseStamped(decision_config.red().reload_point());
        info.opp_shield = Point2PoseStamped(decision_config.red().shield_point());
        info.start_position = info.is_master? Point2PoseStamped(decision_config.blue().master_bot().start_position()):Point2PoseStamped(decision_config.blue().wing_bot().start_position());
    } 
    else{
        info.my_reload = Point2PoseStamped(decision_config.red().reload_point());
        info.my_shield = Point2PoseStamped(decision_config.red().shield_point());
        info.opp_reload = Point2PoseStamped(decision_config.blue().reload_point());
        info.opp_shield = Point2PoseStamped(decision_config.blue().shield_point());
        info.start_position = info.is_master? Point2PoseStamped(decision_config.red().master_bot().start_position()): Point2PoseStamped(decision_config.red().wing_bot().start_position());
        
    }


    // use wifi
    if (decision_config.usewifi()){
        if (info.is_master){
            std::string ip = decision_config.master_ip();
            std::string port = "";
            int length = ip.length();
            for (int i=length-1; i>=0; i--){
                if (ip[i] != ':')
                    port = ip[i] + port;
                else
                    break; 
            }
            port = "tcp://*:"  + port;
            masterSocket_.bind(port);
            masterThread = std::thread(&Blackboard::CommunicateMaster, this);
        }
        else{
            clientSocket_.connect(decision_config.master_ip());
            clientThread = std::thread(&Blackboard::CommunicateClient, this);
        }
    }


  }

  ~Blackboard() = default;


  // supply request
  void SupplyRequest(int number){
      printf("Supply Request!");
      roborts_msgs::ProjectileSupply msg;
    //   if (number==50){
    //       info.times_to_supply -= 1; 
    //   }
    //   else{
    //       info.times_to_supply = 0;
    //   }
      info.is_supplying = true;
      msg.number = number;
      supply_pub_.publish(msg);
  }
 

  void Shoot(int hz){
     if (!is_in_shoot_state && info.can_shoot){
        is_in_shoot_state = true;
        shoot_thread = std::thread(&Blackboard::_ShootThread, this, hz);
     }
  }


  void CShoot(){
      if (info.can_shoot){
         roborts_msgs::ShooterCmd cmd;
         cmd.is_shoot = true;
         cmd.shoot_cmd = 0;
         cmd.c_shoot_cmd = 1;
         cmd.shoot_freq = 2500;
         shoot_pub_.publish(cmd);
      }
      
  }

  void StopShoot(){
      if (is_in_shoot_state){
        is_in_shoot_state = false;
        roborts_msgs::ShooterCmd cmd;
        cmd.is_shoot = false;
        cmd.shoot_cmd = 0;
        cmd.c_shoot_cmd = 0;
        cmd.shoot_freq = 2500;
        shoot_pub_.publish(cmd);
        shoot_thread.join();
      }  
  }

  void StartDodge(){
      if (info.can_dodge){
          roborts_msgs::DodgeMode dg;
          dg.is_dodge = true;
          dodge_pub_.publish(dg);
          in_dodge = true;
      }
      
  }

  void StopDodge(){
      roborts_msgs::DodgeMode dg;
      dg.is_dodge = false;
      dodge_pub_.publish(dg);
      in_dodge = false;
  }
  


  // Laser Armor Detection
  void ArmorCallback(const roborts_msgs::ArmorsPos::ConstPtr &armors){
        // #printf("Laser Dtected!\n");
        auto curPose = GetRobotMapPose();
        bool valid[2] = {false, false};

        info.valid_camera_armor = info.valid_front_camera_armor || info.valid_back_camera_armor;
        

        // Remove the invalid armor infos
        if (armors->num_armor >0){
            float cur_x, cur_y;
            for (int i=0; i< armors->num_armor; i++){
                if (i==0){
                    cur_x = armors->armor_0[0];
                    cur_y = armors->armor_0[1];
                }
                else{
                    cur_x = armors->armor_1[0];
                    cur_y = armors->armor_1[1];
                }

            // in Ally points
            if (GetEulerDistance(cur_x, cur_y, info.ally.pose.position.x, info.ally.pose.position.y) <= 0.75 )    continue;
            
            
            // // cut out dispatch and out of bound.
            // else  if ((cur_x >=2.4 && cur_x <=3.4  && cur_y>=4.5)||
            //          (cur_x >= 8 || cur_x<=0 || cur_y>=5 || cur_y<=0)) continue;
            
            // in death points
            else{ 
                bool flag = false;
                for (auto it=death_Points.begin(); it !=death_Points.end(); it++){
                    geometry_msgs::Point deadPoint = *it;
                    double distance =  GetEulerDistance(cur_x, cur_y, deadPoint.x, deadPoint.y);
                    if (distance<= threshold.near_dist && !info.valid_front_camera_armor){
                        flag = true;
                        std::cout<<"ID:"<< i<<" Death_Points: " <<deadPoint.x <<", "  << deadPoint.y << std::endl;
                        break;
                    }
                }
                if (flag)  continue;

                if (use_camera){
                auto curGimbalPose = GetRobotGimbalMapPose();
                    geometry_msgs::PoseStamped toEnemy;
                    toEnemy.pose.orientation = GetRelativeQuaternion(cur_x, cur_y, curGimbalPose);
                    static int invalid_count[2] ={0};

                    double delta_angle1 = std::abs(GetAngle(toEnemy, curGimbalPose));
                    double delta_angle2 = std::abs(GetAngle(toEnemy, curPose));
                
                    if ( (delta_angle1<= threshold.near_angle || delta_angle2 <= threshold.near_angle) && !info.valid_front_camera_armor){
                        invalid_count[i]++;
                        if (invalid_count[i] >10){
                            geometry_msgs::Point p;
                            p.x = cur_x;
                            p.y = cur_y;
                            p.z = 0;
                            death_Points.push_back(p);
                            if (death_Points.size()>6) {death_Points.pop_front();}
                            invalid_count[i] = 0;
                            continue;

                        }   

                    }
                    else{
                        invalid_count[i] = 0;
                    }

                }
                

            }
            // valid index
            valid[i] = true;

            }
                        
        }
        else{
            info.has_first_enemy = _front_first_enemy || _back_first_enemy;
            info.has_second_enemy = _front_second_enemy || _back_second_enemy;
            info.has_my_enemy = info.has_first_enemy || info.has_second_enemy;
            return;
        }
        

        std::set<int> valid_set;
        if (valid[0]==true || valid[1] == true){
            // After Filtering the valid armor.
            info.has_my_enemy = true;
            if (valid[0]){
                double e_pos_x, e_pos_y;
                e_pos_x = armors->armor_0[0];
                e_pos_y = armors->armor_0[1];
                // printf("valid[0]:(%f, %f) ", e_pos_x, e_pos_y);
                double dist1 = GetEulerDistance(e_pos_x, e_pos_y, info.first_enemy.pose.position.x, info.first_enemy.pose.position.y);
                double dist2 = GetEulerDistance(e_pos_x, e_pos_y, info.second_enemy.pose.position.x, info.second_enemy.pose.position.y);
                if (dist1 <= dist2){
                    info.has_first_enemy = true;
                    info.first_enemy.pose.position.x = e_pos_x;
                    info.first_enemy.pose.position.y = e_pos_y;
                    valid_set.insert(1);
                }
                else{
                    info.has_second_enemy = true;
                    info.second_enemy.pose.position.x = e_pos_x;
                    info.second_enemy.pose.position.y = e_pos_y;
                    valid_set.insert(2);
                }
            }
            if (valid[1]){
                double e_pos_x, e_pos_y;
                e_pos_x = armors->armor_1[0];
                e_pos_y = armors->armor_1[1];
                // printf("valid[1]:(%f, %f)", e_pos_x, e_pos_y);
                double dist1 = GetEulerDistance(e_pos_x, e_pos_y, info.first_enemy.pose.position.x, info.first_enemy.pose.position.y);
                double dist2 = GetEulerDistance(e_pos_x, e_pos_y, info.second_enemy.pose.position.x, info.second_enemy.pose.position.y);
                if (dist1 <= dist2){
                    info.has_first_enemy = true;
                    info.first_enemy.pose.position.x = e_pos_x;
                    info.first_enemy.pose.position.y = e_pos_y;
                    valid_set.insert(1);
                }
                else{
                    info.has_second_enemy = true;
                    info.second_enemy.pose.position.x = e_pos_x;
                    info.second_enemy.pose.position.y = e_pos_y;
                    valid_set.insert(2);
                }
            }
            
            // clear invalid id
            if (valid_set.find(1) == valid_set.end())  info.has_first_enemy = _back_first_enemy;
            if (valid_set.find(2) == valid_set.end())  info.has_second_enemy = _back_second_enemy;
        }
        else{
            info.has_first_enemy = _front_first_enemy || _back_first_enemy;
            info.has_second_enemy = _front_second_enemy || _back_second_enemy;
            info.has_my_enemy = info.has_first_enemy || info.has_second_enemy;
            return;
        }
        
        
    
  }

  // Back Camera Armor Detection
  void BackCameraArmorCallback(const roborts_msgs::ArmorsPos::ConstPtr &armors){
      //   printf("Front Camera!\n");
      if (armors->num_armor == 0){
          info.valid_back_camera_armor = false;
          _back_first_enemy = false;
          _back_second_enemy = false;
      }
      else{
        // Set up valid set
        std::set<int> valid_id_set; 
        bool valid = false;
        for (int i=0; i<armors->num_armor; i++){
            int id = armors->id[i];
            int state = armors->state[i];
            valid_id_set.insert(id);  // ????!!!!!!!!!!!!!!!!!!! IS ID 0 all dealth cars
            if (i == 0){
                bool cur_state = bool(state == 1);
                // printf("ID0\n");
                if (cur_state && armors->pose_A.size()>0){
                    double ex,ey;
                    ex = armors->pose_A[0];
                    ey = armors->pose_A[1];
                    if (ex<=0 || ex>=8 || ey<=0 || ey>=5)
                        continue;

                    if (id == 1){
                        _back_first_enemy = true;
                        info.first_enemy.pose.position.x = ex;
                        info.first_enemy.pose.position.y = ey;
                    }
                    else if (id == 2){
                        _back_second_enemy = true;
                        info.second_enemy.pose.position.x = ex;
                        info.second_enemy.pose.position.y = ey;
                    } 
                    else if (id==0 || id==3){ // do not know which car should be update, then according to history
                        double dist1 = GetEulerDistance(ex, ey, info.first_enemy.pose.position.x, info.first_enemy.pose.position.y);
                        double dist2 = GetEulerDistance(ex, ey, info.second_enemy.pose.position.x, info.second_enemy.pose.position.y);
                        if (dist1<=dist2){
                            _back_first_enemy = true;
                            info.first_enemy.pose.position.x = ex;
                            info.first_enemy.pose.position.y = ey;
                            valid_id_set.insert(1);
                        }
                        else{
                            _back_second_enemy = true;
                            info.second_enemy.pose.position.x = ex;
                            info.second_enemy.pose.position.y = ey;
                            valid_id_set.insert(2);
                        }

                    }
                    valid = true;
                }
            }
            else if (i == 1){
                bool cur_state = bool(state == 1);
                //  printf("ID1\n");
                if (cur_state && armors->pose_B.size()>0){
                    double ex,ey;
                    ex = armors->pose_B[0];
                    ey = armors->pose_B[1];
                    if (ex<=0 || ex>=8 || ey<=0 || ey>=5)
                        continue;

                    if (id == 1){
                        _back_first_enemy = true;
                        info.first_enemy.pose.position.x = ex;
                        info.first_enemy.pose.position.y = ey;
                    }
                    else if (id == 2){
                        _back_second_enemy = true;
                        info.second_enemy.pose.position.x = ex;
                        info.second_enemy.pose.position.y = ey;
                    }
                    else if (id == 0 || id==3){
                        double dist1 = GetEulerDistance(ex, ey, info.first_enemy.pose.position.x, info.first_enemy.pose.position.y);
                        double dist2 = GetEulerDistance(ex, ey, info.second_enemy.pose.position.x, info.second_enemy.pose.position.y);
                        if (dist1<=dist2){
                            _back_first_enemy = true;
                            info.first_enemy.pose.position.x = ex;
                            info.first_enemy.pose.position.y = ey;
                            valid_id_set.insert(1);
                        }
                        else{
                            _back_second_enemy = true;
                            info.second_enemy.pose.position.x = ex;
                            info.second_enemy.pose.position.y = ey;
                            valid_id_set.insert(2);
                        }
                    }   
                    valid = true;
                }
            }
        }
        
        // all 0.
        if (!valid){
            info.valid_back_camera_armor = false;
        }
        else{
            info.valid_back_camera_armor = true;
        }
        
        // Clear invalid set
        for (int id=1; id<3; id++){
            if (valid_id_set.find(id) == valid_id_set.end()){
                if (id == 1) _back_first_enemy = false;
                else if (id == 2) _back_second_enemy = false;
            }
        }
          

            
      }
  }

  // Front Camera Armor Detection
  void FrontCameraArmorCallback(const roborts_msgs::ArmorsPos::ConstPtr &armors){
    //   printf("Front Camera!\n");
      
      if (armors->num_armor == 0){
          info.valid_front_camera_armor = false;
          _front_first_enemy = false;
          _front_second_enemy = false;
      }
      else{
        
            // Set up valid set
            std::set<int> valid_id_set; 
            bool valid = false;
            for (int i=0; i<armors->num_armor; i++){
                int id = armors->id[i];
                int state = armors->state[i];
                valid_id_set.insert(id); // ?????????????????!!!!!!!!!!!!!!!   is ID 0 should be all dealth points???
                if (i == 0){
                    bool cur_state = bool(state == 1);
                    // printf("ID0\n");
                    if (cur_state && armors->pose_A.size()>0){
                        double ex,ey;
                        ex = armors->pose_A[0];
                        ey = armors->pose_A[1];
                        if (ex<=0 || ex>=8 || ey<=0 || ey>=5)
                           continue;

                        if (id == 1){
                            _front_first_enemy = true;
                            info.first_enemy.pose.position.x = ex;
                            info.first_enemy.pose.position.y = ey;
                        }
                        else if (id == 2){
                            _front_second_enemy = true;
                            info.second_enemy.pose.position.x = ex;
                            info.second_enemy.pose.position.y = ey;
                        } 
                        else if (id==0 || id==3){ // do not know which car should be update, then according to history
                            double dist1 = GetEulerDistance(ex, ey, info.first_enemy.pose.position.x, info.first_enemy.pose.position.y);
                            double dist2 = GetEulerDistance(ex, ey, info.second_enemy.pose.position.x, info.second_enemy.pose.position.y);
                            if (dist1<=dist2){
                                _front_first_enemy = true;
                                info.first_enemy.pose.position.x = ex;
                                info.first_enemy.pose.position.y = ey;
                                valid_id_set.insert(1);
                            }
                            else{
                                _front_second_enemy = true;
                                info.second_enemy.pose.position.x = ex;
                                info.second_enemy.pose.position.y = ey;
                                valid_id_set.insert(2);
                            }

                        }
                        valid = true;
                    }
                }
                else if (i == 1){
                    bool cur_state = bool(state == 1);
                    //  printf("ID1\n");
                    if (cur_state && armors->pose_B.size()>0){
                        double ex,ey;
                        ex = armors->pose_B[0];
                        ey = armors->pose_B[1];
                        if (ex<=0 || ex>=8 || ey<=0 || ey>=5)
                           continue;

                        if (id == 1){
                            _front_first_enemy = true;
                            info.first_enemy.pose.position.x = ex;
                            info.first_enemy.pose.position.y = ey;
                        }
                        else if (id == 2){
                            _front_second_enemy = true;
                            info.second_enemy.pose.position.x = ex;
                            info.second_enemy.pose.position.y = ey;
                        }
                        else if (id == 0 || id==3){
                            double dist1 = GetEulerDistance(ex, ey, info.first_enemy.pose.position.x, info.first_enemy.pose.position.y);
                            double dist2 = GetEulerDistance(ex, ey, info.second_enemy.pose.position.x, info.second_enemy.pose.position.y);
                            if (dist1<=dist2){
                                _front_first_enemy = true;
                                info.first_enemy.pose.position.x = ex;
                                info.first_enemy.pose.position.y = ey;
                                valid_id_set.insert(1);
                            }
                            else{
                                _front_second_enemy = true;
                                info.second_enemy.pose.position.x = ex;
                                info.second_enemy.pose.position.y = ey;
                                valid_id_set.insert(2);
                            }
                        }   
                        valid = true;
                    }
                }
            }
            
            // all 0.
            if (!valid){
                info.valid_front_camera_armor = false;
            }
            else{
                info.valid_front_camera_armor = true;
            }
         
            // Clear invalid set
            for (int id=1; id<3; id++){
                if (valid_id_set.find(id) == valid_id_set.end()){
                    if (id == 1) _front_first_enemy = false;
                    else if (id == 2) _front_second_enemy = false;
                }
            }
          

            
      }
     
      
  }

  // Is here right in recieved bullet???????????????!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // FieldSUpply Callback
  void FieldSupplyCallback(const roborts_msgs::SupplierStatus::ConstPtr &msg){
      static unsigned int count = 0;
      unsigned int status = msg->status;
      if (status == 1){
          info.is_supplying = true;
          count = 0;
      }
      else if (info.is_supplying  && status==0){
              count++;
      }

      if (count>=3){
          count = 0;
          info.is_supplying = false;
      }

  }
 
 // Buff Callback
 void BuffCallback(const roborts_msgs::BonusStatus::ConstPtr &msg){
    if (info.team_blue)  info.has_buff = bool(msg->blue_bonus ==2);
    else info.has_buff = bool(msg->red_bonus == 2);

    
 }
  
  // vel call back
  void VelAccCallback(const roborts_msgs::TwistAccel::ConstPtr &msg){
      my_vel_x = msg->twist.linear.x;
      my_vel_y = msg->twist.linear.y;

  }
  
  // Game Status call back
  void GameStatusCallback(const roborts_msgs::GameStatus::ConstPtr &msg){
      info.remaining_time = msg->remaining_time;
      info.is_begin = bool(msg->game_status == 4);
      info.game_status = msg->game_status;
  }

  // Cmd Gimbal call back
  void CmdGimbalCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg){
        bool yaw_mode = msg->yaw_mode;
        bool pitch_mode = msg->pitch_mode;
        float yaw_angle = msg->yaw_angle;
        float pitch_angle = msg->pitch_angle;
        float cal_yaw_angle = msg->cal_yaw_angle;
        if (yaw_mode && std::abs(yaw_angle) <= 0.2 && info.valid_front_camera_armor){
             _gimbal_can = true;
        }
        else{
            _gimbal_can = false;
        }
        // gimbal pitch
        // printf("yaw_mode:%d yaw_angle:%f, pitch_mode:%d, pitch_angle:%f\n", yaw_mode, yaw_angle, pitch_mode, pitch_angle);
      
  }



  // Fusion Detection call back
  void FusionDetectionCallback(const roborts_msgs::FusionTarget::ConstPtr &fs){
        num_id_target = fs->num_id_target;
        num_no_id_target = fs->num_no_id_target;
        unsigned int bound = 2 <num_id_target? 2:num_id_target;
        for (int i=0; i<bound; i++) id_targets[i] = fs->id_targets[i];
        bound = 4 <num_no_id_target? 4: num_no_id_target;
        for (int i=0; i<bound; i++) no_id_targets[i] = fs->no_id_targets[i];
   
  }

  // Robot status call back
  void RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr &rb_status){
      static unsigned int count=0;
      info.remain_hp = rb_status->remain_hp;
    //   info.team_blue = bool(rb_status->id == 13 || rb_status->id==14);
      if (info.remain_hp < remain_hp - 50){
          info.is_hitted = true;
          count = 0;
          remain_hp = info.remain_hp;
      }
      else{
          if (count < 300) count++;
          else  info.is_hitted = false;
      }

         
  }
  
  // Robot Heat call back
  void RobotHeatCallback(const roborts_msgs::RobotHeat::ConstPtr &rh){
      info.heat = rh->shooter_heat;
  }

  // Robot Shoot call back
  void RobotShootCallback(const roborts_msgs::RobotShoot::ConstPtr &msg){
      info.frequency = msg->frequency;
      info.speed = msg->speed;
     
  }

  

  // Enemy  // messages are always zero for enemy.
  void ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback){
     
     
    if (feedback->detected){
      info.valid_front_camera_armor = true;
      enemy_detected_ = true;
      // ROS_INFO("Find Enemy!");

      tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
      geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;
      camera_pose_msg = feedback->enemy_pos;

      double distance = std::sqrt(camera_pose_msg.pose.position.x * camera_pose_msg.pose.position.x +
          camera_pose_msg.pose.position.y * camera_pose_msg.pose.position.y);
      double yaw = atan(camera_pose_msg.pose.position.y / camera_pose_msg.pose.position.x);

      //camera_pose_msg.pose.position.z=camera_pose_msg.pose.position.z;
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,
                                                              0,
                                                              yaw);
      camera_pose_msg.pose.orientation.w = quaternion.w();
      camera_pose_msg.pose.orientation.x = quaternion.x();
      camera_pose_msg.pose.orientation.y = quaternion.y();
      camera_pose_msg.pose.orientation.z = quaternion.z();
      poseStampedMsgToTF(camera_pose_msg, tf_pose);
     
      tf_pose.stamp_ = ros::Time(0);
      try
      {
        tf_ptr_->transformPose("map", tf_pose, global_tf_pose);
        tf::poseStampedTFToMsg(global_tf_pose, global_pose_msg);

        if(GetDistance(global_pose_msg, enemy_pose_)>0.2 || GetAngle(global_pose_msg, enemy_pose_) > 0.2){
          enemy_pose_ = global_pose_msg;

        }
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("tf error when transform enemy pose from camera to map");
      }
    } else{
      info.valid_front_camera_armor = false;
      enemy_detected_ = false;
    }



  }

  
  geometry_msgs::PoseStamped GetEnemy() {  // const: Can not introduce New Arguments
    geometry_msgs::PoseStamped cur_pose = GetRobotMapPose();
    geometry_msgs::PoseStamped enemyPose;
    unsigned int shoot_1_cnt = my_shoot_1_cnt + ally_shoot_1_cnt;
    unsigned int shoot_2_cnt = my_shoot_2_cnt + ally_shoot_2_cnt;

    // 2. fixed  1 enemy and attack
    // The different mark
    if (info.has_first_enemy && info.has_second_enemy){
        if (shoot_1_cnt >= shoot_2_cnt){
            enemyPose = info.first_enemy;
            info.last_enemy = enemyPose;
            if (CanShoot()) my_shoot_1_cnt++;
        }
        else{
            enemyPose = info.second_enemy;
            info.last_enemy = enemyPose;
            if (CanShoot()) my_shoot_2_cnt++;
        }
    }
    else if (info.has_first_enemy && info.has_ally_second_enemy){
        if (shoot_1_cnt >= shoot_2_cnt){
            enemyPose = info.first_enemy;
            info.last_enemy = enemyPose;
            if (CanShoot()) my_shoot_1_cnt++;
        }
        else{
            enemyPose = info.ally_second_enemy;
            info.last_enemy = enemyPose;
            if (CanShoot()) my_shoot_2_cnt++;
        }
    }
    else if (info.has_ally_first_enemy && info.has_second_enemy ){
        if (shoot_1_cnt >= shoot_2_cnt){
            enemyPose = info.ally_first_enemy;
            info.last_enemy = enemyPose;
            if (CanShoot()) my_shoot_1_cnt++;
        }
        else{
            enemyPose = info.second_enemy;
            info.last_enemy = enemyPose;
            if (CanShoot()) my_shoot_2_cnt++;
        }
    }
    else if (info.has_ally_first_enemy && info.has_ally_second_enemy){
        if (shoot_1_cnt >= shoot_2_cnt){
            enemyPose = info.ally_first_enemy;
            info.last_enemy = enemyPose;
            if (CanShoot()) my_shoot_1_cnt++;
        }
        else{
            enemyPose = info.ally_second_enemy;
            info.last_enemy = enemyPose;
            if (CanShoot()) my_shoot_2_cnt++;
        }
    }
    // Then, my first enemy
    else if (info.has_first_enemy){
         enemyPose = info.first_enemy;
         info.last_enemy = enemyPose;
         if (CanShoot()) my_shoot_1_cnt++;
    }
    else if (info.has_second_enemy){
         enemyPose = info.second_enemy;
         info.last_enemy = enemyPose;
         if (CanShoot()) my_shoot_2_cnt++;
    }
    else if (info.has_ally_first_enemy){
          enemyPose = info.ally_first_enemy;
          info.last_enemy = enemyPose;
          if (CanShoot()) my_shoot_1_cnt++;
    }
    else if (info.has_ally_second_enemy){
          enemyPose = info.ally_second_enemy;
          info.last_enemy = enemyPose;
          if (CanShoot()) my_shoot_2_cnt++;
    }
    else {
        enemyPose = info.last_enemy;
    } 




    // 1. Select near first enemy
    // // my detect enemy
    // if (info.has_my_enemy){
    //      if (info.has_first_enemy && info.has_second_enemy){
    //           if (shoot_1_cnt >= shoot_2_cnt){
    //               enemyPose = info.first_enemy;
    //               info.last_enemy = enemyPose;
    //               if (CanShoot()){
    //                   my_shoot_1_cnt++;
    //               }
                  
    //           }
    //           else{
    //               enemyPose = info.second_enemy;
    //               info.last_enemy = enemyPose;
    //               if (CanShoot()){
    //                  my_shoot_2_cnt++;
    //               }
                  
    //           }
    //     //     double dist1 = GetEulerDistance(cur_pose.pose.position.x, cur_pose.pose.position.y, info.first_enemy.pose.position.x, info.first_enemy.pose.position.y);
    //     //     double dist2 = GetEulerDistance(cur_pose.pose.position.x, cur_pose.pose.position.y, info.second_enemy.pose.position.x, info.second_enemy.pose.position.y);
    //     //   if (dist1<=dist2){
    //     //       enemyPose = info.first_enemy;
    //     //       info.last_enemy = enemyPose;
    //     //   }
    //     //   else{
    //     //       enemyPose = info.second_enemy;
    //     //       info.last_enemy = enemyPose;
              
    //     //   }
            
    //     }
    //     else if (info.has_first_enemy){
    //         enemyPose = info.first_enemy;
    //         info.last_enemy = enemyPose;
    //         if (CanShoot()){
    //            my_shoot_1_cnt++;
    //         }
            
    //     }
    //     else if (info.has_second_enemy){
    //         enemyPose = info.second_enemy;
    //         info.last_enemy = enemyPose;
    //         if (CanShoot()){
    //             my_shoot_2_cnt++;
    //         }
            
    //     }
            
    // }

    // // ally detect enemy
    // else if (info.has_ally_enemy){
    //     if (info.has_ally_first_enemy && info.has_ally_second_enemy){
    //          if (shoot_1_cnt >= shoot_2_cnt){
    //               enemyPose = info.ally_first_enemy;
    //               info.last_enemy = enemyPose;
    //           }
    //           else{
    //               enemyPose = info.ally_second_enemy;
    //               info.last_enemy = enemyPose;
    //           }

    //     //     double dist1 = GetEulerDistance(cur_pose.pose.position.x, cur_pose.pose.position.y, info.ally_first_enemy.pose.position.x, info.ally_first_enemy.pose.position.y);
    //     //     double dist2 = GetEulerDistance(cur_pose.pose.position.x, cur_pose.pose.position.y, info.ally_second_enemy.pose.position.x, info.ally_second_enemy.pose.position.y);
    //     //   if (dist1<=dist2){
    //     //       enemyPose = info.ally_first_enemy;
    //     //       info.last_enemy = enemyPose;
    //     //   }
    //     //   else{
    //     //       enemyPose = info.ally_second_enemy;
    //     //       info.last_enemy = enemyPose;
    //     //   }
            
    //     }
    //     else if (info.has_ally_first_enemy){
    //         enemyPose = info.ally_first_enemy;
    //         info.last_enemy = enemyPose;
    //     }
    //     else if (info.has_ally_second_enemy){
    //         enemyPose = info.ally_second_enemy;
    //         info.last_enemy = enemyPose;
    //     }
           

    // }
    // else {
    //     enemyPose = info.last_enemy;
    // }
    
    
    enemyPose.pose.orientation = GetRelativeQuaternion(enemyPose, cur_pose);
    info.last_enemy = enemyPose;
    info.got_last_enemy = true;
    
    return enemyPose;

    

    
  }

  bool CanShoot(){
       if (_gimbal_can                    // gimbal yaw can
           && info.heat <threshold.heat_upper_bound   // heater bound
           && ( (my_vel_x==0 && my_vel_y ==0) || in_dodge )  // velocity or dodge
           && info.valid_front_camera_armor)   // valid front camera
          return true;
       else{
           StopShoot();
           return false;
       }
  }


  bool CanDodge(){
      // whether dodge in my reload!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      geometry_msgs::PoseStamped myPose = GetRobotMapPose();
      // relative yaw angle
      if (!_gimbal_can 
         || !info.is_hitted
         || (GetDistance(info.my_reload, myPose) <=0.65 && !_dodge_in_reload)
         || (info.remain_bullet <0 && info.times_to_supply>0)
      )
      {
          StopDodge();
          return false;
      }

      geometry_msgs::PoseStamped enemyPose = GetEnemy();
      
      // absolute distance
      double distance = GetDistance(myPose, enemyPose);
      if (distance >2.0 && distance<0.5 && info.remain_bullet >= 1000){
          StopDodge();
          return false;
      }

    // // Stop Dodge Condition in Costmap.
    //   const double radius = 0.125;
    //   const double sqrt2 = 1.414;
    //   double x,y;
    //   double cur_x, cur_y;
    //   double cost;
    //   double int_x[] = {-radius, -radius/sqrt2, 0, radius/sqrt2, radius, radius/sqrt2, 0, -radius/sqrt2};
    //   double int_y[] = {0, -radius/sqrt2, -radius, radius/sqrt2, 0, radius/sqrt2, radius, -radius/sqrt2};
    //   int u_x, u_y;
    //   x = myPose.pose.position.x;
    //   y = myPose.pose.position.y;
    //   for (int i=0; i<=7; i++){
    //         cur_x = x + int_x[i];
    //         cur_y = y + int_y[i];
    //         GetCostMap2D()->World2MapWithBoundary(cur_x, cur_y, u_x, u_y);
    //         cost = GetCostMap2D()->GetCost(u_x, u_y);
    //         if (cost >=253){
    //             StopDodge();
    //             return false;
    //         }
                      
          
    //   }
      return true;

  }

  bool IsEnemyDetected() const{
    ROS_INFO("%s: %d", __FUNCTION__, (int)enemy_detected_);
    return enemy_detected_;
  }

  // Goal
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){
    new_goal_ = true;
    goal_ = *goal;
  }

  geometry_msgs::PoseStamped GetGoal() const {
    return goal_;
  }

   geometry_msgs::PoseStamped GetMyGoal() const {
    return info.my_goal;
  }
  
  void SetMyGoal(geometry_msgs::PoseStamped goal){
      info.my_goal = goal;
  }
  
  bool IsNewGoal(){
    if(new_goal_){
      new_goal_ =  false;
      return true;
    } else{
      return false;
    }
  }


  bool IsBombAllyGoal(const geometry_msgs::PoseStamped goal){
      auto distance = GetDistance(goal, info.ally_goal);
      return distance < threshold.near_dist && !info.is_master;
  }

  bool hasOtherUnitsInThisArea(const geometry_msgs::PoseStamped target){
      std::vector<geometry_msgs::PoseStamped>  others;
      double dist;
      others.push_back(info.ally);
      if (info.has_first_enemy)
          others.push_back(info.first_enemy);
      if (info.has_second_enemy)
          others.push_back(info.second_enemy);
      if (info.has_ally_first_enemy)
          others.push_back(info.ally_first_enemy);
      if (info.has_ally_second_enemy)
          others.push_back(info.ally_second_enemy);
      for (int i=0; i<others.size(); i++){
         dist = GetDistance(target, others[i]);
         if (dist <= threshold.near_dist)
            return true;
      }
      return false;
      
  }

 

  bool IsBombAllyGoal(const float x, const float y){
      auto distance = GetEulerDistance(x, y, info.ally_goal.pose.position.x, info.ally_goal.pose.position.y);
      return distance < threshold.near_dist;
  }


  bool IsInStuckArea(){
      geometry_msgs::PoseStamped curPose = GetRobotMapPose();
      double yaw = tf::getYaw(curPose.pose.orientation);
      float cur_x = curPose.pose.position.x, cur_y = curPose.pose.position.y;
      float x[4], y[4];
      float deg_30 = 30 / 180 * 3.1415926;
      float d = 0.3;
      x[0] = std::max(0.0, std::min(7.99, cur_x + d * std::cos(yaw + deg_30)));
      y[0] = std::max(0.0, std::min(4.99, cur_y + d * std::sin(yaw + deg_30)));
      x[1] = std::max(0.0, std::min(7.99, cur_x + d * std::cos(yaw - deg_30)));
      y[1] = std::max(0.0, std::min(4.99, cur_y + d * std::sin(yaw - deg_30)));
      x[2] = std::max(0.0, std::min(7.99, cur_x - d * std::cos(yaw + deg_30)));
      y[2] = std::max(0.0, std::min(4.99, cur_y - d * std::sin(yaw + deg_30)));
      x[3] = std::max(0.0, std::min(7.99, cur_x - d * std::cos(yaw - deg_30)));
      y[3] = std::max(0.0, std::min(4.99, cur_y - d * std::sin(yaw - deg_30)));
      unsigned int mx, my;
      for (int i=0; i<4; i++){
          GetCostMap2D()->World2Map(x[i], y[i], mx, my);
          if (GetCostMap2D()->GetCost(mx, my) <253)
            return false;

      }

      // In stuck area
      
      const double sqrt2 = 1.414;
      double cost;
      double c_x, c_y;
      double int_x[] = {-d, -d/sqrt2, 0, d/sqrt2, d, d/sqrt2, 0, -d/sqrt2};
      double int_y[] = {0, -d/sqrt2, -d, d/sqrt2, 0, d/sqrt2, d, -d/sqrt2};
      int u_x, u_y;
      double acc_angle = 0.0;
      double cur_angle = 0.0;
      double count = 0;
      for (int i=0; i<=7; i++){
            c_x = cur_x + int_x[i];
            c_y = cur_y + int_y[i];
            GetCostMap2D()->World2MapWithBoundary(c_x, c_y, u_x, u_y);
            cost = GetCostMap2D()->GetCost(u_x, u_y);
            if (cost >= 253){
                cur_angle = std::atan2(int_y[i], int_x[i] + 0.00001);
                cur_angle = cur_angle >0? cur_angle : cur_angle + 2 * 3.1415926;
                acc_angle += cur_angle;
                count++;
            }
                      
          
      }
      acc_angle = acc_angle / count;
      acc_angle = acc_angle - 3.1415926;
      acc_angle = acc_angle < -3.1415926 ? acc_angle + 2 * 3.1415926 : acc_angle;

      // in case of rotate in a roll.
      c_x = cur_x + d * std::cos(acc_angle);
      c_y = cur_y + d * std::sin(acc_angle);
      GetCostMap2D()->World2MapWithBoundary(c_x, c_y, u_x, u_y);
      cost = GetCostMap2D()->GetCost(u_x, u_y);
      if (cost >=253){
          acc_angle = acc_angle >0 ? acc_angle - 3.1415926 : acc_angle + 3.1415926;
      }
    //   printf("acc_angle: %f\n", acc_angle * 180 / 3.14);

      // get yaw
      acc_angle = acc_angle - yaw;
      double vx, vy;
      vx = cos(acc_angle);
      vy = sin(acc_angle);
      geometry_msgs::Twist tw;
      tw.linear.x = vx;
      tw.linear.y = vy;
      tw.linear.z = 0;
      // has sent cmd_vel
      cmd_vel_pub_.publish(tw);
      
     

      return true;
  }




  /*---------------------------------- Tools ------------------------------------------*/

  geometry_msgs::Quaternion GetRelativeQuaternion(const geometry_msgs::PoseStamped to,const geometry_msgs::PoseStamped from){
      double dy, dx;
      dy = to.pose.position.y - from.pose.position.y;
      dx = to.pose.position.x - from.pose.position.x;
      double yaw = std::atan2(dy, dx);    
      return tf::createQuaternionMsgFromYaw(yaw);
  }

  geometry_msgs::Quaternion GetRelativeQuaternion(const double to_x, const double to_y, const geometry_msgs::PoseStamped from){
      double dx, dy;
      dy = to_y - from.pose.position.y;
      dx = to_x - from.pose.position.x;
      double yaw = std::atan2(dy, dx);
      return tf::createQuaternionMsgFromYaw(yaw);
  }

  double GetDistance(const geometry_msgs::PoseStamped &pose1,
                     const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Point point1 = pose1.pose.position;
    const geometry_msgs::Point point2 = pose2.pose.position;
    const double dx = point1.x - point2.x;
    const double dy = point1.y - point2.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double GetEulerDistance(const float x1, const float y1, const float x2, const float y2){
     return std::sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
  }

  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
    const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(quaternion1, rot1);
    tf::quaternionMsgToTF(quaternion2, rot2);
    return rot1.angleShortestPath(rot2);
  }

  geometry_msgs::PoseStamped Point2PoseStamped(Point point){
      geometry_msgs::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.pose.position.x = point.x();
      ps.pose.position.y = point.y();
      ps.pose.position.z = point.z();
      tf::Quaternion q = tf::createQuaternionFromRPY(point.roll(),
                                                   point.pitch(),
                                                   point.yaw());
      ps.pose.orientation.x = q.x();
      ps.pose.orientation.y = q.y();
      ps.pose.orientation.z = q.z();
      ps.pose.orientation.w = q.w();
      return ps;
  }

  const geometry_msgs::PoseStamped GetRobotMapPose() {
    UpdateRobotPose();
    return self_pose_;
  }

  const geometry_msgs::PoseStamped GetRobotGimbalMapPose(){
    UpdateRobotGimbalPose();
    return gimbal_pose_;
  }

  const std::shared_ptr<CostMap> GetCostMap(){
    return costmap_ptr_;
  }

  const CostMap2D* GetCostMap2D() {
    return costmap_2d_;
  }

  const unsigned char* GetCharMap() {
    return charmap_;
  }


  DecisionInfoPool info;
  Threshold threshold;

 private:
  ComInfo setComInfo(){
      ComInfo ci;
      ci.times_to_supply = info.times_to_supply;
      ci.times_to_buff = info.times_to_buff;
      ci.hp = info.remain_hp;
      ci.shoot_1 = my_shoot_1_cnt;
      ci.shoot_2 = my_shoot_2_cnt;
      ci.has_enemy = info.has_my_enemy;
      ci.bullet = info.remain_bullet;
      ci.pose_x = self_pose_.pose.position.x;
      ci.pose_y = self_pose_.pose.position.y;
      ci.yaw = tf::getYaw(self_pose_.pose.orientation);
      ci.first_valid = info.has_first_enemy;
      ci.first_enemy_x = info.first_enemy.pose.position.x;
      ci.first_enemy_y = info.first_enemy.pose.position.y;
      
      ci.second_valid = info.has_second_enemy;
      ci.second_enemy_x = info.second_enemy.pose.position.x;
      ci.second_enemy_y = info.second_enemy.pose.position.y;
      
      ci.goal_x = info.my_goal.pose.position.x;
      ci.goal_y = info.my_goal.pose.position.y;

      // fusion info
      ci.fusion.num_id_target = num_id_target;
      ci.fusion.num_no_id_target = num_no_id_target;
      unsigned int bound = 2 <num_id_target?2:num_id_target;
      for (int i=0; i<bound; i++) ci.fusion.id_targets[i] = id_targets[i];
      bound = 4 < num_no_id_target? 4:num_no_id_target;
      for (int i=0; i<bound; i++) ci.fusion.no_id_targets[i] = no_id_targets[i];
      return ci;
  }

  void getComInfo(ComInfo ci){
 
      if (info.remaining_time % 60 <= 1){
          info.times_to_buff = 1;
          info.times_to_supply = 2;
      }
      else{
          info.times_to_supply = std::min(info.times_to_supply, ci.times_to_supply);
          info.times_to_buff = std::min(info.times_to_buff, ci.times_to_buff);
      }
      
      info.ally_remain_hp = ci.hp;
      info.ally_remain_bullet = ci.bullet;
      info.ally.header.stamp = ros::Time::now();
      info.ally.pose.position.x = ci.pose_x;
      info.ally.pose.position.y = ci.pose_y;
      info.ally.pose.position.z = 0;
      info.ally.pose.orientation = tf::createQuaternionMsgFromYaw(ci.yaw);
      info.has_ally_first_enemy = ci.first_valid;
      if (ci.first_valid){ 
          info.ally_first_enemy.pose.position.x = ci.first_enemy_x;
          info.ally_first_enemy.pose.position.y = ci.first_enemy_y;
      }
      info.has_ally_second_enemy = ci.second_valid;
      if (ci.second_valid) {
          info.ally_second_enemy.pose.position.x = ci.second_enemy_x;
          info.ally_second_enemy.pose.position.y = ci.second_enemy_y;
      }
      info.ally_goal.pose.position.x = ci.goal_x;
      info.ally_goal.pose.position.y = ci.goal_y;
      info.has_ally_enemy = ci.has_enemy;

    // fusion info
      ally_num_id_target = ci.fusion.num_id_target;
      ally_num_no_id_target = ci.fusion.num_no_id_target;
      unsigned int bound = 2 <ally_num_id_target? 2:ally_num_id_target;
      for (int i=0; i<bound; i++)  ally_id_targets[i] = ci.fusion.id_targets[i];
      bound = 4 < ally_num_no_id_target? 4 :ally_num_no_id_target;
      for (int i=0; i<bound; i++)  ally_no_id_targets[i] = ci.fusion.no_id_targets[i];

      
    // prepare to publish fusion data
      FS.id_targets.clear();
      FS.no_id_targets.clear();
      FS.num_id_target = ally_num_id_target;
      FS.num_no_id_target = ally_num_no_id_target;
      for (int i=0; i< ally_num_id_target; i++) FS.id_targets.push_back(ally_id_targets[i]);
      for (int i=0; i< ally_num_no_id_target; i++) FS.no_id_targets.push_back(ally_no_id_targets[i]);

      // ally shoot cnt
      ally_shoot_1_cnt = ci.shoot_1;
      ally_shoot_2_cnt = ci.shoot_2;


      
  }

  // timeit shoot bullet control
  void _ShootThread(int hz){
      ros::Rate loop(hz);
      while (ros::ok()){
          roborts_msgs::ShooterCmd cmd;
          cmd.is_shoot = true;
          cmd.shoot_cmd = 1;
          cmd.c_shoot_cmd = 0;
          cmd.shoot_freq = 2500;
          shoot_pub_.publish(cmd);
          info.remain_bullet = info.remain_bullet - 1;
          if (! is_in_shoot_state)
             break;
          loop.sleep();
          
      }
      
  }


  // master and client communication function
  void CommunicateMaster(){
    ComInfo ci;
    ros::Rate loop(50);
    while (ros::ok()){
        zmq::message_t rec_message;
        masterSocket_.recv(&rec_message);
        memcpy(&ci, rec_message.data(), sizeof(ci));
        getComInfo(ci);   
        //---------------------------------------------------------------------------------------------------------------------------------------
        ci = setComInfo();
        zmq::message_t send_message(sizeof(ci));
        memcpy(send_message.data(), &ci, sizeof(ci));
        
        masterSocket_.send(send_message);

        ally_pub_.publish(info.ally);
        fusion_pub_.publish(FS);
        info.has_ally = true;
       
        loop.sleep();
    }
  }

  void CommunicateClient(){
    
    ComInfo ci;
    ros::Rate loop(50);

    while (ros::ok()){

        ci = setComInfo();
        zmq::message_t send_message(sizeof(ci));
        memcpy(send_message.data(), &ci, sizeof(ci));
        
        clientSocket_.send(send_message);
        //----------------------------------------------------------------------------------------------------------------------------------------
        
        zmq::message_t rec_message;
        clientSocket_.recv(&rec_message);
        memcpy(&ci, rec_message.data(), sizeof(ci));
        getComInfo(ci);
        
        geometry_msgs::PoseStamped ap;
       
        ally_pub_.publish(info.ally);
        fusion_pub_.publish(FS);
        info.has_ally = true;
       
        loop.sleep();       
    }
  }




  void UpdateRobotPose() {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();
     
    robot_tf_pose.frame_id_ = pose_frameID;
    robot_tf_pose.stamp_ = ros::Time();
    try {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, self_pose_);
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }


  void UpdateRobotGimbalPose(){
    if (use_camera){
        tf::Stamped<tf::Pose> gimbal_tf_pose;
        gimbal_tf_pose.setIdentity();

        gimbal_tf_pose.frame_id_ = gimbal_frameID;
        gimbal_tf_pose.stamp_ = ros::Time();
        try{
            geometry_msgs::PoseStamped gimbal_pose;
            tf::poseStampedTFToMsg(gimbal_tf_pose, gimbal_pose);
            tf_ptr_->transformPose("map", gimbal_pose, gimbal_pose_);

        }
        catch (tf::LookupException &ex){
            ROS_ERROR("Transform Error looking up gimbal pose: %s", ex.what());
        }
    }
    
  }

  geometry_msgs::PoseStamped InitMapPose(){
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "map";
      p.pose.position.x = -99;
      p.pose.position.y = -99;
      return p;
  }


  //! list<death points>
  std::list<geometry_msgs::Point> death_Points;


  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //! Enenmy detection
  ros::Subscriber enemy_sub_;
  ros::Subscriber armor_sub_, camera_armor_sub_, back_camera_sub_, info_sub_, fusion_target_sub_, cmd_gimbal_sub_;
  ros::Subscriber robot_status_sub_, robot_shoot_sub_, robot_heat_sub_, game_status_sub_, supply_sub_, buff_sub_, vel_acc_sub_;
  // publisher
  ros::Publisher shoot_pub_, ally_pub_, fusion_pub_, dodge_pub_, supply_pub_, cmd_vel_pub_;

  //! Goal info
  geometry_msgs::PoseStamped goal_;
  bool new_goal_;

  //! Enemy info
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  roborts_msgs::ArmorDetectionGoal armor_detection_goal_;
  geometry_msgs::PoseStamped enemy_pose_;
  bool enemy_detected_;

  //! cost map
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D* costmap_2d_;
  unsigned char* charmap_;

  //! robot map pose
  geometry_msgs::PoseStamped self_pose_, enemy_;
  //! robot gimbal pose
  geometry_msgs::PoseStamped gimbal_pose_;

  // point
  geometry_msgs::Point camera_enemy_;

  //remain hp
  int remain_hp;


  // zmq
  zmq::context_t context_;
  zmq::socket_t masterSocket_, clientSocket_;
  std::thread masterThread, clientThread;
  std::thread shoot_thread;
  bool is_in_shoot_state;
 
  


  // frame id
  std::string pose_frameID, gimbal_frameID;
  bool use_camera;

  // fusion
  unsigned int num_id_target, ally_num_id_target;
  unsigned int num_no_id_target, ally_num_no_id_target;
  unsigned int my_shoot_1_cnt, my_shoot_2_cnt, ally_shoot_1_cnt, ally_shoot_2_cnt;
  roborts_msgs::Target id_targets[2], ally_id_targets[2];
  roborts_msgs::Target no_id_targets[4], ally_no_id_targets[4];
  roborts_msgs::FusionTarget FS;
  double my_vel_x, my_vel_y;

  bool _gimbal_can, in_dodge;
  
  bool _front_first_enemy, _front_second_enemy;
  bool _back_first_enemy, _back_second_enemy;
  bool _dodge_in_reload;
};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H
