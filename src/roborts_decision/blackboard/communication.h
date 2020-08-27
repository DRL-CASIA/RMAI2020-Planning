#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include "roborts_msgs/Target.h"



struct Detections{
    int num_id_target;
    int num_no_id_target;
    roborts_msgs::Target id_targets[2];
    roborts_msgs::Target no_id_targets[4];
};

struct ComInfo{
    int times_to_supply;
    int times_to_buff;
    int hp;
    int shoot_1, shoot_2;
    int bullet;
    float pose_x, pose_y, yaw;
    bool has_enemy;
    bool first_valid;
    float first_enemy_x, first_enemy_y;
    bool second_valid;
    float second_enemy_x, second_enemy_y;
    float goal_x, goal_y;
    Detections fusion;
    
};


#endif //COMMUNICATION_H