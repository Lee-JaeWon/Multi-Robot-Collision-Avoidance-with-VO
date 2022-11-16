#!/usr/bin/env python3

"""
author : LeeJaewon
email : jawwoni@naver.com
GitHub : Lee-JaeWon
"""

from math import sqrt
import rospy
from geometry_msgs.msg import PoseStamped 
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

class get_global_path():
    def __init__(self):
        rospy.init_node('path_pub')
        self.namespace = rospy.get_param('~robot')
        
        print(self.namespace)
        # self.namespace = 'tb3_0'
        rospy.loginfo_once("Recived namespace : " + self.namespace)
        self.pub_name = "marker/" + self.namespace + "/gpp"
        self.marker_pub = rospy.Publisher(self.pub_name, Marker, queue_size=10) # marker publish for Visualization
        self.poses_x = None
        self.poses_y = None
        
    def ggp_callback(self, data):
        self.poses_x = [x.pose.position.x for x in data.poses]
        self.poses_y = [y.pose.position.y for y in data.poses]
        if self.poses_x is None:
            rospy.loginfo("waiting for plan data")

    def path_sub(self):
        # self.ps_sub_name = self.namespace + "/move_base/NavfnROS/plan"
        self.ps_sub_name = self.namespace + "/astar_nav_path"
        self.ps_sub = rospy.Subscriber(self.ps_sub_name, Path, self.ggp_callback, queue_size=1)

    def get_path(self):
        while not rospy.is_shutdown():
            self.path_sub()
            self.pos_marker()
            if self.poses_x is not None:
                rospy.loginfo(self.namespace + "'s Path is received")
                for i in range(len(self.poses_x)):
                    self.pose_points.points.append(Point(self.poses_x[i], self.poses_y[i], 0))
                self.marker_pub.publish(self.pose_points)

    def pos_marker(self):
        self.pose_points = Marker()

        self.pose_points.header.frame_id = "map"
        self.pose_points.header.stamp=rospy.Time.now()
        self.pose_points.ns = "points"
        self.pose_points.id = 0
        self.pose_points.type = Marker.POINTS
        self.pose_points.action = Marker.ADD
        self.pose_points.pose.orientation.w = 1.0
        self.pose_points.scale.x = 0.2
        self.pose_points.scale.y = 0.2
        if self.namespace == 'tb3_0':
            self.pose_points.color = ColorRGBA(1, 0, 1, 1)
        elif self.namespace == 'tb3_1':
            self.pose_points.color = ColorRGBA(1, 1, 0, 1)
        elif self.namespace == 'tb3_2':
            self.pose_points.color = ColorRGBA(0, 1, 1, 1)
        self.pose_points.lifetime == rospy.Duration()

if __name__ == '__main__':
    _obj = get_global_path()
    _obj.get_path()

