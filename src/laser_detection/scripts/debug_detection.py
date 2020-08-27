#!/usr/bin/env python
import rospy
from messages.msg import EnemyPos
from messages.msg import RoboState
from messages.msg import ArmorPos
import matplotlib.pyplot as plt
import numpy as np
from math import *

class Visualize():
    def __init__(self):

        self.control_yaw = []
        self.control_pitch = []
        self.control_time = []
        self.buffer_size = 30

        self.predict_time = []
        self.predict_yaw = []

        self.armor_pos_time = []
        self.enemy_x_in_gimbal = []
        self.enemy_y_in_gimbal = []

        self.enemy_x_in_map = []
        self.enemy_y_in_map = []
        self.enemy_z_in_map = []
        self.enemy_vx_in_map = []
        self.enemy_vy_in_map = []
        self.enemy_distance = []

        self.enemy_yaw_in_gimbal = []
        self.enemy_pitch_in_gimbal = []
        self.gimbal_yaw = []
        self.gimbal_pitch = []
        self.control_sub = rospy.Subscriber("enemy_pos", EnemyPos, self.ControlCallback)
        self.state_sub = rospy.Subscriber("robo_state", RoboState, self.RoboStateCallback)
        self.armor_sub = rospy.Subscriber("laser_armor", ArmorPos, self.ArmorPoseCallback)


    def ControlCallback(self, data):
        time = rospy.Time.now().to_sec()
        self.control_time.append(time)
        self.control_yaw.append(data.enemy_yaw)
        self.control_pitch.append(data.enemy_pitch)
        if len(self.control_yaw) > self.buffer_size:
            self.control_yaw = self.control_yaw[-self.buffer_size:]
            self.control_pitch = self.control_pitch[-self.buffer_size:]
            self.control_time = self.control_time[-self.buffer_size:]
        self.PredictYaw(time)

    def PredictYaw(self, time):
        if len(self.control_yaw) >=10:
            histroy_yaw = np.asarray(self.control_yaw[-10:]).reshape(-1, 1)
            histroy_time = np.asarray(self.control_time[-10:]).reshape(-1, 1)
            # print histroy_time.shape
            t0 = histroy_time[0]
            time = time - t0
            histroy_time = histroy_time - t0
            X = np.concatenate(( histroy_time, np.ones_like(histroy_time)), axis=1)
            W = np.dot(np.dot(np.linalg.inv(np.dot(X.transpose(), X)), X.transpose()), histroy_yaw)
            # print W.shape
            p_x = np.array([ time, 1])
            p_yaw = np.dot(p_x, W)
            self.predict_time.append(time + t0)
            self.predict_yaw.append(p_yaw[0])

            if len(self.predict_time) > self.buffer_size:
                self.predict_time = self.predict_time[-self.buffer_size:]
                self.predict_yaw = self.predict_yaw[-self.buffer_size:]


    def RoboStateCallback(self, data):
        self.gimbal_yaw.append(data.gimbal_yaw)
        self.gimbal_pitch.append(data.gimbal_pitch)

    def ArmorPoseCallback(self, data):
        self.armor_pos_time.append(rospy.Time.now().to_sec())
        if data.is_valid:
            self.enemy_x_in_map.append(data.x)
            self.enemy_y_in_map.append(data.y)
            self.enemy_z_in_map.append(data.z)

            self.enemy_x_in_gimbal.append(data.pose_in_gimbal[0])
            self.enemy_y_in_gimbal.append(data.pose_in_gimbal[1])
            self.enemy_distance.append(data.distance)
            if data.pose_in_gimbal[0] > 1e-4:
                enemy_pitch_in_gimbal = atan(data.pose_in_gimbal[1] / data.pose_in_gimbal[0])
                enemy_yaw_in_gimbal = atan(-data.pose_in_gimbal[2] / data.pose_in_gimbal[0])
                self.enemy_pitch_in_gimbal.append(enemy_pitch_in_gimbal)
                self.enemy_yaw_in_gimbal.append(enemy_yaw_in_gimbal)
            else:
                self.enemy_yaw_in_gimbal.append(0)
                self.enemy_pitch_in_gimbal.append(0)
            self.enemy_vx_in_map.append(data.motion_vec[0] / 0.1)
            self.enemy_vy_in_map.append(data.motion_vec[1] / 0.1)
        else:
            self.enemy_pitch_in_gimbal.append(0)
            self.enemy_yaw_in_gimbal.append(0)
            self.enemy_x_in_map.append(-3)
            self.enemy_y_in_map.append(-3)
            self.enemy_z_in_map.append(-1)
            self.enemy_x_in_gimbal.append(-1)
            self.enemy_y_in_gimbal.append(-1)
            self.enemy_distance.append(-1)
            self.enemy_vx_in_map.append(-1)
            self.enemy_vy_in_map.append(-1)


if __name__ == "__main__":
    rospy.init_node("debug_node", anonymous=True)
    debug_show = Visualize()
    update = rospy.Rate(10)
    while not rospy.is_shutdown():
        plt.subplot(4, 1, 1)
        plt.plot(debug_show.armor_pos_time, debug_show.enemy_distance)
        plt.ylim((-1, 8))
        plt.grid()
        plt.subplot(4, 1, 2)

        plt.plot(debug_show.armor_pos_time, debug_show.enemy_x_in_map, '+-')
        plt.plot(debug_show.armor_pos_time, debug_show.enemy_y_in_map, '+-')
        plt.plot(debug_show.armor_pos_time, debug_show.enemy_z_in_map)

        plt.ylim((-3, 8))
        plt.grid()
        plt.subplot(4, 1, 3)
        plt.plot(debug_show.armor_pos_time, debug_show.enemy_x_in_gimbal)
        plt.plot(debug_show.armor_pos_time, debug_show.enemy_y_in_gimbal)
        plt.ylim((-8, 8))
        plt.grid()
        plt.subplot(4, 1, 4)
        # plt.plot(debug_show.armor_pos_time, debug_show.enemy_vx_in_map)
        # plt.plot(debug_show.armor_pos_time, debug_show.enemy_vy_in_map)
        # plt.ylim(-0.5, 0.5)
        # plt.plot(debug_show.gimbal_yaw)
        plt.plot(debug_show.control_time, debug_show.control_yaw)
        plt.plot(debug_show.control_time, debug_show.control_pitch)
        # if(len(debug_show.predict_time) == len(debug_show.predict_yaw)):
        #     plt.plot(debug_show.predict_time, debug_show.predict_yaw, '.-')
        plt.grid()
        plt.draw()
        plt.pause(0.1)
        plt.clf()

