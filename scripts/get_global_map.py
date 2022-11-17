#!/usr/bin/env python3
#-*- coding: utf-8 -*-

"""
author : LeeJaewon
email : jawwoni@naver.com
GitHub : Lee-JaeWon
"""

import rospy
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx
import matplotlib.patches as patches
from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from math import pi, sin, cos, sqrt, atan2, asin, acos, degrees
import time
import threading


class get_global_map():
    def __init__(self):
        rospy.init_node('get_map_data')
        self.map_topic = "/map"
        rospy.loginfo_once(self.map_topic)
        rospy.loginfo_once("-- get map node and VO --")

        # Robot Data Topic
        self.mapData = None
        self.posData_1 = None
        self.posData_2 = None
        self.posData_3 = None
        self.poseData_1 = None
        self.poseData_2 = None
        self.poseData_3 = None
        self.cmdData_1 = None
        self.cmdData_2 = None
        self.cmdData_3 = None
        self.goal_1 = None
        self.goal_2 = None
        self.goal_3 = None
        self.quat_Data_1 = None
        self.quat_Data_2 = None
        self.quat_Data_3 = None
        self.path_1_x = None
        self.path_1_y = None
        self.path_2_x = None
        self.path_2_y = None
        self.path_3_x = None
        self.path_3_y = None
        self.pose_topic_1 = "/yaw_0"
        self.pose_topic_2 = "/yaw_1"
        self.pose_topic_3 = "/yaw_2"
        self.cmd_topic_1 = '/tb3_0/cmd_vel'
        self.cmd_topic_2 = '/tb3_1/cmd_vel'
        self.cmd_topic_3 = '/tb3_2/cmd_vel'
        self.pos_topic_1 = "/multi_position_0"
        self.pos_topic_2 = "/multi_position_1"
        self.pos_topic_3 = "/multi_position_2"
        self.goal_topic_1 = "/tb3_0/move_base_my/goal"
        self.goal_topic_2 = "/tb3_1/move_base_my/goal"
        self.goal_topic_3 = "/tb3_2/move_base_my/goal"
        self.quat_topic_1 = "/quat_0"
        self.quat_topic_2 = "/quat_1"
        self.quat_topic_3 = "/quat_2"
        self.path_topic_1 = "/tb3_0/astar_nav_path"
        self.path_topic_2 = "/tb3_1/astar_nav_path"
        self.path_topic_3 = "/tb3_2/astar_nav_path"

        # Robot Spec
        self.num_robots = 3
        # After new Mapping, Change This
        self.X = [[33.0, 37.0], [36.0, 55.0], [80.0, 34.0]]
        self.P = [[0.0, 0.0] for _ in range(self.num_robots)]
        self.temp_V = [[0.0, 0.0] for _ in range(self.num_robots)]
        self.V = [[0, 0] for _ in range(3)]
        self.ws_model = dict()
        self.ws_model['robot_radius'] = 4.0

        # Robot Data Subscriber
        # map
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.mapCallback)
        # position
        rospy.Subscriber(self.pos_topic_1, Point, self.posiCallback_1)
        rospy.Subscriber(self.pos_topic_2, Point, self.posiCallback_2)
        rospy.Subscriber(self.pos_topic_3, Point, self.posiCallback_3)
        # pose_yaw
        rospy.Subscriber(self.pose_topic_1, Float32, self.poseCallback_1)
        rospy.Subscriber(self.pose_topic_2, Float32, self.poseCallback_2)
        rospy.Subscriber(self.pose_topic_3, Float32, self.poseCallback_3)
        # cmd_vel : Subscribe
        rospy.Subscriber(self.cmd_topic_1, Twist, self.cmd_vel_Callback_1)
        rospy.Subscriber(self.cmd_topic_2, Twist, self.cmd_vel_Callback_2)
        rospy.Subscriber(self.cmd_topic_3, Twist, self.cmd_vel_Callback_3)
        # Goal Point : Subscribe
        rospy.Subscriber(self.goal_topic_1, PoseStamped, self.goalCallback_1)
        rospy.Subscriber(self.goal_topic_2, PoseStamped, self.goalCallback_2)
        rospy.Subscriber(self.goal_topic_3, PoseStamped, self.goalCallback_3)
        # Quaternion
        rospy.Subscriber(self.quat_topic_1, Quaternion, self.QuatCallback_1)
        rospy.Subscriber(self.quat_topic_2, Quaternion, self.QuatCallback_2)
        rospy.Subscriber(self.quat_topic_3, Quaternion, self.QuatCallback_3)
        # Result Path from Astar
        rospy.Subscriber(self.path_topic_1, Path, self.PathCallback_1)
        rospy.Subscriber(self.path_topic_2, Path, self.PathCallback_2)
        rospy.Subscriber(self.path_topic_3, Path, self.PathCallback_3)

        # cmd_vel : Publisher
        self.tb0_cmd_vel = rospy.Publisher(
            "/tb3_0/cmd_vel", Twist, queue_size=10)
        self.tb1_cmd_vel = rospy.Publisher(
            "/tb3_1/cmd_vel", Twist, queue_size=10)
        self.tb2_cmd_vel = rospy.Publisher(
            "/tb3_2/cmd_vel", Twist, queue_size=10)
        self.tb0_PWCM_pub = rospy.Publisher(
            "/tb3_0/my_initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.tb1_PWCM_pub = rospy.Publisher(
            "/tb3_1/my_initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.tb2_PWCM_pub = rospy.Publisher(
            "/tb3_2/my_initialpose", PoseWithCovarianceStamped, queue_size=10)

        self.tb0_PWCM_obj = PoseWithCovarianceStamped()
        self.tb1_PWCM_obj = PoseWithCovarianceStamped()
        self.tb2_PWCM_obj = PoseWithCovarianceStamped()

        self.robot_patch = [None]*self.num_robots
        self.goal_patch = [None]*self.num_robots

        # For Velocity Obstacle
        self.vmax = [0.05 for _ in range(self.num_robots)]
        self.V_des = [[0.0, 0.0] for _ in range(self.num_robots)]
        self._VO = VO(VMAX=self.vmax)
        self.res_V = [[0.0, 0.0] for _ in range(self.num_robots)]
        self.ip = [[0] for _ in range(self.num_robots)]
        self.op = [[0] for _ in range(self.num_robots)]
        self.degree = [[0] for _ in range(self.num_robots)]

        # My Math Tool
        self.math = Math_calc()
        self.PI = pi

        # Goal for VO
        self.goal = [[83.0, 53.0], [80.0, 31.0], [39.0, 57.0]]
        self.way_point = [[0.0, 0.0] for _ in range(self.num_robots)]

        # For translate vel vector to v,w
        self.dt = 0.005
        self.omega = [[0.0] for _ in range(self.num_robots)]
        self.pre_degree = [0]*self.num_robots
        self.tb3_0_twist = Twist()
        self.tb3_1_twist = Twist()
        self.tb3_2_twist = Twist()

        self.reach = [False, False, False]

    # Map
    def mapCallback(self, data):
        self.mapData = data

    # Goal
    def goalCallback_1(self, data):
        self.goal_1 = data

    def goalCallback_2(self, data):
        self.goal_2 = data

    def goalCallback_3(self, data):
        self.goal_3 = data

    # Position
    def posiCallback_1(self, data):
        self.posData_1 = data

    def posiCallback_2(self, data):
        self.posData_2 = data

    def posiCallback_3(self, data):
        self.posData_3 = data

    # yaw
    def poseCallback_1(self, data):
        self.poseData_1 = data

    def poseCallback_2(self, data):
        self.poseData_2 = data

    def poseCallback_3(self, data):
        self.poseData_3 = data

    def cmd_vel_Callback_1(self, data):
        self.cmdData_1 = data

    def cmd_vel_Callback_2(self, data):
        self.cmdData_2 = data

    def cmd_vel_Callback_3(self, data):
        self.cmdData_3 = data

    def QuatCallback_1(self, data):
        self.quat_Data_1 = data

    def QuatCallback_2(self, data):
        self.quat_Data_2 = data

    def QuatCallback_3(self, data):
        self.quat_Data_3 = data

    # Path
    def PathCallback_1(self, data):
        self.path_1_x = [x.pose.position.x for x in data.poses]
        self.path_1_y = [y.pose.position.y for y in data.poses]

    def PathCallback_2(self, data):
        self.path_2_x = [x.pose.position.x for x in data.poses]
        self.path_2_y = [y.pose.position.y for y in data.poses]

    def PathCallback_3(self, data):
        self.path_3_x = [x.pose.position.x for x in data.poses]
        self.path_3_y = [y.pose.position.y for y in data.poses]

    def make_PWCM(self):  # For PoseWithCovarianceStamped
        if self.posData_1 is not None and self.quat_Data_1 is not None:
            self.tb0_PWCM_obj.header.frame_id = "map"
            self.tb0_PWCM_obj.header.stamp = rospy.get_rostime()
            self.tb0_PWCM_obj.pose.pose.position.x = self.posData_1.x
            self.tb0_PWCM_obj.pose.pose.position.y = self.posData_1.y
            self.tb0_PWCM_obj.pose.pose.position.z = self.posData_1.z
            self.tb0_PWCM_obj.pose.pose.orientation.x = self.quat_Data_1.x
            self.tb0_PWCM_obj.pose.pose.orientation.y = self.quat_Data_1.y
            self.tb0_PWCM_obj.pose.pose.orientation.z = self.quat_Data_1.z
            self.tb0_PWCM_obj.pose.pose.orientation.w = self.quat_Data_1.w
            self.tb0_PWCM_pub.publish(self.tb0_PWCM_obj)
        if self.posData_2 is not None and self.quat_Data_2 is not None:
            self.tb1_PWCM_obj.header.frame_id = "map"
            self.tb1_PWCM_obj.header.stamp = rospy.get_rostime()
            self.tb1_PWCM_obj.pose.pose.position.x = self.posData_2.x
            self.tb1_PWCM_obj.pose.pose.position.y = self.posData_2.y
            self.tb1_PWCM_obj.pose.pose.position.z = self.posData_2.z
            self.tb1_PWCM_obj.pose.pose.orientation.x = self.quat_Data_2.x
            self.tb1_PWCM_obj.pose.pose.orientation.y = self.quat_Data_2.y
            self.tb1_PWCM_obj.pose.pose.orientation.z = self.quat_Data_2.z
            self.tb1_PWCM_obj.pose.pose.orientation.w = self.quat_Data_2.w
            self.tb1_PWCM_pub.publish(self.tb1_PWCM_obj)
        if self.posData_3 is not None and self.quat_Data_3 is not None:
            self.tb2_PWCM_obj.header.frame_id = "map"
            self.tb2_PWCM_obj.header.stamp = rospy.get_rostime()
            self.tb2_PWCM_obj.pose.pose.position.x = self.posData_3.x
            self.tb2_PWCM_obj.pose.pose.position.y = self.posData_3.y
            self.tb2_PWCM_obj.pose.pose.position.z = self.posData_3.z
            self.tb2_PWCM_obj.pose.pose.orientation.x = self.quat_Data_3.x
            self.tb2_PWCM_obj.pose.pose.orientation.y = self.quat_Data_3.y
            self.tb2_PWCM_obj.pose.pose.orientation.z = self.quat_Data_3.z
            self.tb2_PWCM_obj.pose.pose.orientation.w = self.quat_Data_3.w
            self.tb2_PWCM_pub.publish(self.tb2_PWCM_obj)

    def get_data(self):
        figure = plt.figure()
        map_plt = figure.add_subplot(1, 1, 1)

        cmap = self.get_cmap(5)

        while not rospy.is_shutdown():
            # start_time = time.time()
            map_plt.clear()
            if self.mapData is not None:

                data = self.mapData.data
                width = self.mapData.info.width
                height = self.mapData.info.height
                resolution = self.mapData.info.resolution
                self.startx = self.mapData.info.origin.position.x
                self.starty = self.mapData.info.origin.position.y

                if self.cmdData_1 is not None:
                    self.V[0][0] = self.cmdData_1.linear.x
                    self.V[0][1] = self.cmdData_1.angular.z
                if self.cmdData_2 is not None:
                    self.V[1][0] = self.cmdData_2.linear.x
                    self.V[1][1] = self.cmdData_2.angular.z
                if self.cmdData_3 is not None:
                    self.V[2][0] = self.cmdData_3.linear.x
                    self.V[2][1] = self.cmdData_3.angular.z

                # Pose Information
                if self.poseData_1 is not None:
                    self.x_1 = cos(self.poseData_1.data) * \
                        cos(0)
                    self.y_1 = sin(self.poseData_1.data) * \
                        cos(0)
                    self.P[0][0] = self.x_1

                    self.P[0][1] = self.y_1
                if self.poseData_2 is not None:
                    self.x_2 = cos(self.poseData_2.data) * \
                        cos(0)
                    self.y_2 = sin(self.poseData_2.data) * \
                        cos(0)
                    self.P[1][0] = self.x_2
                    self.P[1][1] = self.y_2
                if self.poseData_3 is not None:
                    self.x_3 = cos(self.poseData_3.data) * \
                        cos(0)
                    self.y_3 = sin(self.poseData_3.data) * \
                        cos(0)
                    self.P[2][0] = self.x_3
                    self.P[2][1] = self.y_3

                # Fianl Goal
                if self.goal_1 is not None:
                    self.goal[0][0] = (self.goal_1.pose.position.x -
                                       self.startx)/resolution
                    self.goal[0][1] = (self.goal_1.pose.position.y -
                                       self.starty)/resolution
                if self.goal_2 is not None:
                    self.goal[1][0] = (self.goal_2.pose.position.x -
                                       self.startx)/resolution
                    self.goal[1][1] = (self.goal_2.pose.position.y -
                                       self.starty)/resolution
                if self.goal_3 is not None:
                    self.goal[2][0] = (self.goal_3.pose.position.x -
                                       self.startx)/resolution
                    self.goal[2][1] = (self.goal_3.pose.position.y -
                                       self.starty)/resolution

                # Way Point
                self.sel_way_point = 3
                if self.path_1_x is not None and self.path_1_y is not None:
                    if len(self.path_1_x) > 6:
                        self.way_point[0][0] = (self.path_1_x[self.sel_way_point] -
                                                self.startx)/resolution
                        self.way_point[0][1] = (self.path_1_y[self.sel_way_point] -
                                                self.starty)/resolution
                    else:
                        self.way_point[0][0] = self.goal[0][0]
                        self.way_point[0][1] = self.goal[0][1]

                if self.path_2_x is not None and self.path_2_y is not None:
                    if len(self.path_2_x) > 6:
                        self.way_point[1][0] = (self.path_2_x[self.sel_way_point] -
                                                self.startx)/resolution
                        self.way_point[1][1] = (self.path_2_y[self.sel_way_point] -
                                                self.starty)/resolution
                    else:
                        self.way_point[1][0] = self.goal[1][0]
                        self.way_point[1][1] = self.goal[1][1]

                if self.path_3_x is not None and self.path_3_y is not None:
                    if len(self.path_3_x) > 6:
                        self.way_point[2][0] = (self.path_3_x[self.sel_way_point] -
                                                self.startx)/resolution
                        self.way_point[2][1] = (self.path_3_y[self.sel_way_point] -
                                                self.starty)/resolution
                    else:
                        self.way_point[2][0] = self.goal[2][0]
                        self.way_point[2][1] = self.goal[2][1]
                # print(self.way_point)
                # print(self.path_1_x)
                # if self.path_1_x is not None:
                #     print(len(self.path_1_x), len(self.path_1_y))
                #     print(len(self.path_2_x), len(self.path_2_y))
                #     print(len(self.path_3_x), len(self.path_3_y))
                # print(len(self.path_1_x))
                # Postion Information
                if self.posData_1 is not None:
                    self.X[0][0] = self.posData_1.x
                    self.X[0][1] = self.posData_1.y
                    self.X[0][0] = (self.X[0][0]-self.startx)/resolution
                    self.X[0][1] = (self.X[0][1]-self.starty)/resolution
                if self.posData_2 is not None:
                    self.X[1][0] = self.posData_2.x
                    self.X[1][1] = self.posData_2.y
                    self.X[1][0] = (self.X[1][0]-self.startx)/resolution
                    self.X[1][1] = (self.X[1][1]-self.starty)/resolution
                if self.posData_3 is not None:
                    self.X[2][0] = self.posData_3.x
                    self.X[2][1] = self.posData_3.y
                    self.X[2][0] = (self.X[2][0]-self.startx)/resolution
                    self.X[2][1] = (self.X[2][1]-self.starty)/resolution

                for i in range(self.num_robots):
                    self.robot_patch[i] = patches.Circle((self.X[i][0], self.X[i][1]),
                                                         self.ws_model['robot_radius'],
                                                         facecolor=cmap(i),
                                                         edgecolor='black',
                                                         linewidth=1.5,
                                                         ls='solid',
                                                         alpha=1,
                                                         zorder=2)
                    map_plt.add_patch(self.robot_patch[i])
                    map_plt.arrow(self.X[i][0], self.X[i][1],
                                  100*self.res_V[i][0],
                                  100*self.res_V[i][1],
                                  head_width=2.0, head_length=1.5, fc=cmap(i), ec=cmap(i))

                    # Problem
                    map_plt.arrow(self.X[i][0], self.X[i][1], 10*self.P[i][0], 10*self.P[i]
                                  [1], head_width=2.0, head_length=2.5, fc=cmap(i), ec=cmap(i))

                if self.goal is not None:
                    for i in range(self.num_robots):
                        self.goal_patch[i] = patches.Rectangle((self.goal[i][0], self.goal[i][1]),
                                                               width=4.0, height=4.0, color=cmap(i)
                                                               )
                        map_plt.add_patch(self.goal_patch[i])

                map_plt.set_title("map with localization : " +
                                  str(self.V[0])+str(self.V[1])+str(self.V[2]))

                img = np.zeros((height, width, 3), np.uint8)

                for i in range(0, height):
                    for j in range(0, width):
                        if data[i * width + j] > 60:  # Occupied Score
                            img[i, j] = 0
                        elif data[i * width + j] == 0:  # free space
                            img[i, j] = 255
                        elif data[i * width + j] == -1:  # Unknown Space
                            img[i, j] = 100
                        else:
                            img[i, j] = 50

                for i in range(self.num_robots):
                    self.temp_V[i][0] = self.P[i][0] * self.V[i][0]
                    self.temp_V[i][1] = self.P[i][1] * self.V[i][0]

                if self.way_point[0][0] != 0.0 and self.way_point[0][1] != 0.0:
                    
                    # Velocity Obstacle
                    ## Compute V desired with V maximum
                    self.V_des = self._VO.compute_V_des(
                        X=self.X, way=self.way_point, goal=self.goal)

                    # else:    
                    ## Velocity Update Process
                    self.res_V = self._VO.VO_update(
                    X=self.X, V_des=self.V_des, V_current=self.temp_V, ws_model=self.ws_model, mat_obj=map_plt)
                    
                    for i in range(self.num_robots):
                        if self.res_V[i][0] == 0.0 and self.res_V[i][1] == 0.0:
                            print(f"[Warning] : tb3_{i} has been reached to goal")
                            self.res_V[i][0] = 1e-5
                            self.res_V[i][1] = 1e-5
                            self.reach[i] = True
                        else:
                            self.reach[i] = False
                    print(self.reach)
                    for i in range(self.num_robots):
                        self.ip[i], self.degree[i] = self.math.calc_theta(
                            self.math.unit_vector(self.res_V[i]), self.P[i])
                        self.op[i] = self.math.outer_product(
                            self.math.unit_vector(self.res_V[i]), self.P[i])

                map_plt.imshow(img, cmap='gray', origin='lower')
                plt.draw()
                plt.pause(0.001)

                self.make_PWCM()

                if self.way_point[0][0] != 0.0 and self.way_point[0][1] != 0.0:
                    self.activate(self.degree, self.op, self.res_V, self.reach)
                # print(f"Elapsed Time(While Loop) : {time.time() - start_time}")

    def get_cmap(self, N):
        '''Returns a function that maps each index in 0, 1, ... N-1 to a distinct RGB color.'''
        color_norm = colors.Normalize(vmin=0, vmax=N-1)
        scalar_map = cmx.ScalarMappable(norm=color_norm, cmap='hsv')

        def map_index_to_rgb_color(index):
            return scalar_map.to_rgba(index)
        return map_index_to_rgb_color

    def run(self):
        self.get_data()

    def activate(self, degree, op, res_V, reach):

        angle_gain = 170

        # Way Point가 설정되었다면 linear 부여
        if self.way_point[0][0] != 0.0 and self.way_point[0][1] != 0.0:
            self.tb3_0_twist.linear.x = sqrt(res_V[0][0]**2 + res_V[0][1]**2)
        # Way Point가 설정되지 않았다면 linear 부여 X
        elif reach[0] == True:
            self.tb3_0_twist.linear.x = 0.0
        else:
            self.tb3_0_twist.linear.x = 0.0

        if self.way_point[1][0] != 0.0 and self.way_point[1][1] != 0.0:
            self.tb3_1_twist.linear.x = sqrt(res_V[1][0]**2 + res_V[1][1]**2)
        elif reach[1] == True:
            self.tb3_1_twist.linear.x = 0.0
        else:
            self.tb3_1_twist.linear.x = 0.0

        if self.way_point[2][0] != 0.0 and self.way_point[2][1] != 0.0:
            self.tb3_2_twist.linear.x = sqrt(res_V[1][0]**2 + res_V[1][1]**2)
        elif reach[2] == True:
            self.tb3_2_twist.linear.x = 0.0
        else:
            self.tb3_2_twist.linear.x = 0.0

        if reach[0] == True:
            self.tb3_0_twist.angular.z = 0.0
        else:
            if op[0] > 0:
                    self.tb3_0_twist.angular.z = -(degree[0]/angle_gain)
            else:
                self.tb3_0_twist.angular.z = degree[0]/angle_gain

        if reach[1] == True:
            self.tb3_1_twist.angular.z = 0.0
        else:
            if op[1] > 0:
                    self.tb3_1_twist.angular.z = -(degree[1]/angle_gain)
            else:
                self.tb3_1_twist.angular.z = degree[1]/angle_gain

        if reach[2] == True:
            self.tb3_2_twist.angular.z = 0.0
        else:
            if op[2] > 0:
                    self.tb3_2_twist.angular.z = -(degree[2]/angle_gain)
            else:
                self.tb3_2_twist.angular.z = degree[2]/angle_gain

        self.tb0_cmd_vel.publish(self.tb3_0_twist)
        self.tb1_cmd_vel.publish(self.tb3_1_twist)
        self.tb2_cmd_vel.publish(self.tb3_2_twist)



class VO():
    def __init__(self, VMAX):
        self.vmax = VMAX
        self.PI = pi

    def reach(self, p1, p2, bound=0.5):
        if self.distance(p1, p2) < bound:
            return True
        else:
            return False

    def distance(self, pose1, pose2):
        # compute Euclidean distance for 2D
        return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001

    def compute_V_des(self, X, way, goal):  # V desired
        self.V_des = []
        for i in range(3):
            dif_x = [way[i][k]-X[i][k] for k in range(2)]

            norm = self.distance(dif_x, [0, 0])

            norm_dif_x = [dif_x[k]*self.vmax[k] /
                          norm for k in range(2)]  # Scaling

            self.V_des.append(norm_dif_x[:])

            # When the robot reach to final goal, robot stop.
            if self.reach(X[i], goal[i], 2.5):
                self.V_des[i][0] = 0.0
                self.V_des[i][1] = 0.0
        return self.V_des

    def VO_update(self, X, V_des, V_current, ws_model, mat_obj):
        ROB_RAD = ws_model['robot_radius']+0.1

        V_opt = list(V_current)

        for i in range(3):
            vA = [V_current[i][0], V_current[i][1]]
            pA = [X[i][0], X[i][1]]
            VO_BA_all = []
            for j in range(3):
                if i != j:

                    # Calculate with the others
                    vB = [V_current[j][0], V_current[j][1]]
                    pB = [X[j][0], X[j][1]]

                    # For VO
                    # transl_vB_vA = [pA[0]+vB[0], pA[1]+vB[1]]
                    # For RVO
                    transl_vB_vA = [pA[0]+0.5*(vB[0]+vA[0]), pA[1]+0.5*(vB[1]+vA[1])]

                    # For Visualization
                    mat_obj.plot([X[i][0], X[j][0]], [
                                 X[i][1], X[j][1]], color="green")

                    dist_BA = self.distance(pA, pB)

                    theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])

                    if 2*ROB_RAD > dist_BA:
                        dist_BA = 2*ROB_RAD

                    theta_BAort = asin(2*ROB_RAD/dist_BA)
                    theta_ort_left = theta_BA+theta_BAort
                    bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                    theta_ort_right = theta_BA-theta_BAort
                    bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
                    VO_BA = [transl_vB_vA, bound_left,
                             bound_right, dist_BA, 2*ROB_RAD]
                    VO_BA_all.append(VO_BA)

            vA_post = self.intersect(pA, V_des[i], VO_BA_all)
            V_opt[i] = vA_post[:]
        return V_opt

    def intersect(self, pA, vA, VO_BA_all):
        norm_v = self.distance(vA, [0, 0])
        suitable_V = []
        unsuitable_V = []
        for theta in np.arange(0, 2*self.PI, 0.1):
            for rad in np.arange(0.02, norm_v+0.02, norm_v/5.0):
                new_v = [rad*cos(theta), rad*sin(theta)]
                suit = True
                for RVO_BA in VO_BA_all:
                    p_0 = RVO_BA[0]
                    left = RVO_BA[1]
                    right = RVO_BA[2]
                    dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
                    theta_dif = atan2(dif[1], dif[0])
                    theta_right = atan2(right[1], right[0])
                    theta_left = atan2(left[1], left[0])
                    if self.in_between(theta_right, theta_dif, theta_left):
                        suit = False
                        break
                if suit:
                    suitable_V.append(new_v)
                else:
                    unsuitable_V.append(new_v)
        new_v = vA[:]
        suit = True
        for VO_BA in VO_BA_all:
            p_0 = VO_BA[0]
            left = VO_BA[1]
            right = VO_BA[2]
            dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
            theta_dif = atan2(dif[1], dif[0])
            theta_right = atan2(right[1], right[0])
            theta_left = atan2(left[1], left[0])
            if self.in_between(theta_right, theta_dif, theta_left):
                suit = False
                break
        if suit:
            suitable_V.append(new_v)
        else:
            unsuitable_V.append(new_v)
        #----------------------
        if suitable_V:
            # print 'Suitable found'
            vA_post = min(suitable_V, key=lambda v: self.distance(v, vA))
            new_v = vA_post[:]
            for VO_BA in VO_BA_all:
                p_0 = VO_BA[0]
                left = VO_BA[1]
                right = VO_BA[2]
                dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right[1], right[0])
                theta_left = atan2(left[1], left[0])
        else:
            # print 'Suitable not found'
            tc_V = dict()
            for unsuit_v in unsuitable_V:
                tc_V[tuple(unsuit_v)] = 0
                tc = []
                for VO_BA in VO_BA_all:
                    p_0 = VO_BA[0]
                    left = VO_BA[1]
                    right = VO_BA[2]
                    dist = VO_BA[3]
                    rad = VO_BA[4]
                    dif = [unsuit_v[0]+pA[0]-p_0[0], unsuit_v[1]+pA[1]-p_0[1]]
                    theta_dif = atan2(dif[1], dif[0])
                    theta_right = atan2(right[1], right[0])
                    theta_left = atan2(left[1], left[0])
                    if self.in_between(theta_right, theta_dif, theta_left):
                        small_theta = abs(
                            theta_dif-0.5*(theta_left+theta_right))
                        if abs(dist*sin(small_theta)) >= rad:
                            rad = abs(dist*sin(small_theta))
                        big_theta = asin(abs(dist*sin(small_theta))/rad)
                        dist_tg = abs(dist*cos(small_theta)) - \
                            abs(rad*cos(big_theta))
                        if dist_tg < 0:
                            dist_tg = 0
                        tc_v = dist_tg/self.distance(dif, [0, 0])
                        tc.append(tc_v)
                tc_V[tuple(unsuit_v)] = min(tc)+0.001
            WT = 0.2
            vA_post = min(unsuitable_V, key=lambda v: (
                (WT/tc_V[tuple(v)])+self.distance(v, vA)))
        return vA_post

    def in_between(self, theta_right, theta_dif, theta_left):
        if abs(theta_right - theta_left) <= self.PI:
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        else:
            if (theta_left < 0) and (theta_right > 0):
                theta_left += 2*self.PI
                if theta_dif < 0:
                    theta_dif += 2*self.PI
                if theta_right <= theta_dif <= theta_left:
                    return True
                else:
                    return False
            if (theta_left > 0) and (theta_right < 0):
                theta_right += 2*self.PI
                if theta_dif < 0:
                    theta_dif += 2*self.PI
                if theta_left <= theta_dif <= theta_right:
                    return True
                else:
                    return False


class Math_calc():
    def __init__(self):
        self.a = (1, 1)
        self.b = (1, 1)

    def unit_vector(self, x):
        return (x[0] / self.dist(x), x[1] / self.dist(x))

    def calc_one_vector(self, x1, x2):
        return (x1[0] - x2[0], x1[1] - x2[1])

    def dist(self, v):
        return sqrt(v[0] ** 2 + v[1] ** 2)

    def inner_product(self):
        dist_A = self.dist(self.a)
        dist_B = self.dist(self.b)
        if dist_A == 0:
            dist_A = 1.0
        if dist_B == 0:
            dist_B = 1.0
        ip = self.a[0] * self.b[0] + self.a[1] * self.b[1]
        cost = round(ip / (dist_A * dist_B), 2)

        x = acos(cost)
        deg = degrees(x)
        return ip, deg

    def calc_theta(self, a, b):
        self.a = a
        self.b = b
        return self.inner_product()

    def outer_product(self, a, b):
        return np.cross(a, b)

    def L2_Norm(self, x, y):
        return sqrt((x[0] - y[0])**2 + (x[1] - y[1])**2)


if __name__ == '__main__':
    _map_obj = get_global_map()
    _map_obj.run()
