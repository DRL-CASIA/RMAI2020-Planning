#!/usr/bin/env python
import rospy
from messages.msg import EnemyPos
from messages.msg import RoboState
from messages.msg import ArmorPos
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
import numpy as np
from math import *
import cv2
import tf

from sklearn.cluster import DBSCAN

class PerceptionMap():
    def __init__(self):
        self.resolution = 0.1
        self.laser_x = 0.0
        self.laser_y = 0.0
        self.localization_ready = False
        self.laser_tf_listener_ = tf.TransformListener()
        self.points = None
        self.enemy_points = []
        self.map_origin_x = int(1.2 / self.resolution)
        self.map_origin_y = int(0.8 / self.resolution)
        self.map_file = "/home/drl/ros_codes/RoboRTS/tools/map/icra_0.1.pgm"
        self.map_cv = cv2.imread(self.map_file)
        self.map_matrix = 765 - np.sum(np.asarray(self.map_cv), axis=-1) # 765 = 255*3
        self.map_matrix = self.map_matrix / np.max(self.map_matrix)
        self.static_obstale = 1 - self.map_matrix
        self.probability_map = np.ones_like(self.static_obstale) * 10
        self.map_size_h, self.map_size_w = self.static_obstale.shape
        self.in_map_pc_sub = rospy.Subscriber("in_map_pc", PointCloud2, self.laser_update)

    def laser_update(self, data):
        try:
            (trans, rot) = self.laser_tf_listener_.lookupTransform("map", "laser", rospy.Time(0))
            self.laser_x = trans[0]
            self.laser_y = trans[1]
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        gen = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
        points = []
        for p in gen:
            points.append(p)
        points = np.asarray(points)
        self.points = points[:,:2]
        # points in base_link
        base_link = np.array([self.laser_x, self.laser_y])
        base_link_points = points[:, :2] - base_link
        # print points.shape[0]
        # print np.max(points[:,0]), np.max(points[:, 1])
        current_prob_map = np.ones_like(self.probability_map)
        for i in range(base_link_points.shape[0]):
            p = base_link_points[i]
            # print p
            x = p[0]
            y = p[1]
            ratio = y / x
            x_int = int(x / self.resolution)
            if(x_int > 0):
                x_list = np.arange(0, x_int, 1)
            else:
                x_list = np.arange(x_int, 0, 1)
            y_list = ratio*x_list
            x_list = x_list
            x_list = np.asarray(x_list, dtype=int) + self.map_origin_x + int(base_link[0] / self.resolution)
            y_list = np.asarray(y_list, dtype=int) + self.map_origin_y + int(base_link[1] / self.resolution)
            x_list[x_list<0] = 1
            x_list[x_list>self.map_size_w] = self.map_size_w - 1
            y_list[y_list<0] = 1
            y_list[y_list>self.map_size_h] = self.map_size_h - 1
            # print np.max(x_list), np.max(y_list)
            for t in range(x_list.shape[0]):
                if 0<(self.map_size_h - y_list[t])<self.map_size_h and x_list[t] < self.map_size_w:
                    current_prob_map[self.map_size_h - y_list[t], x_list[t]] = 0
        self.probability_map = self.probability_map * current_prob_map
        self.probability_map += current_prob_map
        self.probability_map = self.probability_map * self.static_obstale
        # self.probability_map = self.probability_map / np.max(self.probability_map)
        self.update_prob_position()

    def update_prob_position(self):
        idy, idx = np.where(self.probability_map == np.max(self.probability_map))
        idx = idx.reshape(-1, 1)
        idy = idy.reshape(-1, 1)
        # print idx
        index = np.concatenate((idx, idy), axis=1)
        # cluster
        db = DBSCAN(eps=2, min_samples=100)
        y_db = db.fit_predict(idx)
        unique_labels = set(y_db)
        self.enemy_points = []
        # print unique_labels
        for l in unique_labels:
            cluster_member = y_db == l
            cluster_points = index[cluster_member]
            center = np.mean(cluster_points, axis=0)
            print center
            # print center
            x = center[0] * 0.1 - 1.2
            y = (self.map_size_h - center[1]) * 0.1 - 0.8
            self.enemy_points.append([x, y])






if __name__=="__main__":
    rospy.init_node("perception_map")
    map = PerceptionMap()
    # rospy.spin()
    while not rospy.is_shutdown():
        plt.imshow(map.static_obstale, cmap=plt.cm.gray, interpolation='nearest', extent=[-1.2, 6.8, -0.8, 4.2])
        plt.imshow(map.probability_map, cmap=plt.cm.viridis, alpha=.9, interpolation='bilinear', extent=[-1.2, 6.8, -0.8, 4.2])
        if map.points is not None:
            plt.plot(map.points[:, 0], map.points[:, 1], 'r.')
        # if len(map.enemy_points)!=0:
        #     points = np.asarray(map.enemy_points)
        #     plt.plot(points[:, 0], points[:, 1], 'bo')
        plt.xlim(-1.4, 7.0)
        plt.ylim(-1.0, 4.4)
        plt.pause(0.1)
        plt.clf()
        # map.probability_map = np.ones_like(map.static_obstale)
