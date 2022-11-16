#!/usr/bin/env python3

"""
author : LeeJaewon
email : jawwoni@naver.com
GitHub : Lee-JaeWon
"""

import rospy
import tf
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion

class get_global_position():
    def __init__(self):
        rospy.init_node('get_postion')
        try:
            self.ns_robot_1 = rospy.get_param('~robot_0') # type : str
            self.ns_robot_2 = rospy.get_param('~robot_1') # type : str
            self.ns_robot_3 = rospy.get_param('~robot_2') # type : str
        except:
            raise Exception("No Robot Data")

        self.tf = tf.TransformListener()
        self.pose_marker_pub_1 = rospy.Publisher("marker/"+self.ns_robot_1+"/position", Marker, queue_size=10) # marker publish for Visualization
        self.pose_marker_pub_2 = rospy.Publisher("marker/"+self.ns_robot_2+"/position", Marker, queue_size=10) # marker publish for Visualization
        self.pose_marker_pub_3 = rospy.Publisher("marker/"+self.ns_robot_3+"/position", Marker, queue_size=10) # marker publish for Visualization

        self.position_tb_0 = rospy.Publisher("/multi_position_0", Point, queue_size=10) # Position publish for VO
        self.position_tb_1 = rospy.Publisher("/multi_position_1", Point, queue_size=10) # Position publish for VO
        self.position_tb_2 = rospy.Publisher("/multi_position_2", Point, queue_size=10) # Position publish for VO

        self.yaw_tb_0 = rospy.Publisher("/yaw_0", Float32, queue_size=10) # Pose publish for VO
        self.yaw_tb_1 = rospy.Publisher("/yaw_1", Float32, queue_size=10) # Pose publish for VO
        self.yaw_tb_2 = rospy.Publisher("/yaw_2", Float32, queue_size=10) # Pose publish for VO

        self.pitch_tb_0 = rospy.Publisher("/pitch_0", Float32, queue_size=10) # Pose publish for VO
        self.pitch_tb_1 = rospy.Publisher("/pitch_1", Float32, queue_size=10) # Pose publish for VO
        self.pitch_tb_2 = rospy.Publisher("/pitch_2", Float32, queue_size=10) # Pose publish for VO

        self.quat_tb_0 = rospy.Publisher("/quat_0", Quaternion, queue_size=10) # Pose publish for VO
        self.quat_tb_1 = rospy.Publisher("/quat_1", Quaternion, queue_size=10) # Pose publish for VO
        self.quat_tb_2 = rospy.Publisher("/quat_2", Quaternion, queue_size=10) # Pose publish for VO

        self.euler_1 = None
        self.euler_2 = None
        self.euler_3 = None
        self.quaternion_1 = None
        self.quaternion_2 = None
        self.quaternion_3 = None
        self.quat_obj_1 = Quaternion()
        self.quat_obj_2 = Quaternion()
        self.quat_obj_3 = Quaternion()

    def get_position(self):
        while not rospy.is_shutdown():

            self.position_marker_1()
            self.position_marker_2()
            self.position_marker_3()

            # get position
            position_1 = None
            position_2 = None
            position_3 = None
            try:
                t1 = self.tf.getLatestCommonTime(self.ns_robot_1 + "/base_footprint", "/map")
                position_1, self.quaternion_1 = self.tf.lookupTransform("/map",self.ns_robot_1 + "/base_footprint", t1)
                self.euler_1 = euler_from_quaternion(quaternion=self.quaternion_1)
            except:
                print("tb3_0 failed tf for get position")
                pass
            try:
                t2 = self.tf.getLatestCommonTime(self.ns_robot_2 + "/base_footprint", "/map")
                position_2, self.quaternion_2 = self.tf.lookupTransform("/map",self.ns_robot_2 + "/base_footprint", t2)
                self.euler_2 = euler_from_quaternion(quaternion=self.quaternion_2)
            except:
                print("tb3_1 failed tf for get position")
                pass
            try:
                t3 = self.tf.getLatestCommonTime(self.ns_robot_3 + "/base_footprint", "/map")
                position_3, self.quaternion_3 = self.tf.lookupTransform("/map",self.ns_robot_3 + "/base_footprint", t3)
                self.euler_3 = euler_from_quaternion(quaternion=self.quaternion_3)
            except:
                print("tb3_2 failed tf for get position")
                pass

            if position_1 is not None and self.euler_1 is not None:
                self.pose_points_1.points.append(Point(position_1[0], position_1[1], 0))
                self.pose_marker_pub_1.publish(self.pose_points_1)
                self.position_tb_0.publish(Point(position_1[0], position_1[1], 0))
                self.yaw_tb_0.publish(self.euler_1[2]) # yaw
                self.pitch_tb_0.publish(self.euler_1[1]) # pitch
            if position_2 is not None and self.euler_2 is not None:
                self.pose_points_2.points.append(Point(position_2[0], position_2[1], 0))
                self.pose_marker_pub_2.publish(self.pose_points_2)
                self.position_tb_1.publish(Point(position_2[0], position_2[1], 0))
                self.yaw_tb_1.publish(self.euler_2[2]) # yaw
                self.pitch_tb_1.publish(self.euler_2[1]) # pitch
            if position_3 is not None and self.euler_3 is not None:
                self.pose_points_3.points.append(Point(position_3[0], position_3[1], 0))
                self.pose_marker_pub_3.publish(self.pose_points_3)
                self.position_tb_2.publish(Point(position_3[0], position_3[1], 0))
                self.yaw_tb_2.publish(self.euler_3[2]) # yaw
                self.pitch_tb_2.publish(self.euler_3[1]) # pitch
            if self.quaternion_1 is not None:
                self.quat_obj_1.x = self.quaternion_1[0]
                self.quat_obj_1.y = self.quaternion_1[1]
                self.quat_obj_1.z = self.quaternion_1[2]
                self.quat_obj_1.w = self.quaternion_1[3]
                self.quat_tb_0.publish(self.quat_obj_1)
            if self.quaternion_2 is not None:
                self.quat_obj_2.x = self.quaternion_2[0]
                self.quat_obj_2.y = self.quaternion_2[1]
                self.quat_obj_2.z = self.quaternion_2[2]
                self.quat_obj_2.w = self.quaternion_2[3]
                self.quat_tb_1.publish(self.quat_obj_2)
            if self.quaternion_3 is not None:
                self.quat_obj_3.x = self.quaternion_3[0]
                self.quat_obj_3.y = self.quaternion_3[1]
                self.quat_obj_3.z = self.quaternion_3[2]
                self.quat_obj_3.w = self.quaternion_3[3]
                self.quat_tb_2.publish(self.quat_obj_3)
            

    def position_marker_1(self):
        self.pose_points_1 = Marker()
        self.pose_points_1.header.frame_id = 'map'
        self.pose_points_1.header.stamp=rospy.Time.now()
        self.pose_points_1.ns = "points"
        self.pose_points_1.id = 1
        self.pose_points_1.type = Marker.POINTS
        self.pose_points_1.action = Marker.ADD
        self.pose_points_1.pose.orientation.w = 1.0
        self.pose_points_1.scale.x = 0.3
        self.pose_points_1.scale.y = 0.3
        self.pose_points_1.color = ColorRGBA(1, 0, 0, 1)
        self.pose_points_1.lifetime == rospy.Duration()

    def position_marker_2(self):
        self.pose_points_2 = Marker()
        self.pose_points_2.header.frame_id = 'map'
        self.pose_points_2.header.stamp=rospy.Time.now()
        self.pose_points_2.ns = "points"
        self.pose_points_2.id = 2
        self.pose_points_2.type = Marker.POINTS
        self.pose_points_2.action = Marker.ADD
        self.pose_points_2.pose.orientation.w = 1.0
        self.pose_points_2.scale.x = 0.3
        self.pose_points_2.scale.y = 0.3
        self.pose_points_2.color = ColorRGBA(0, 1, 0, 1)
        self.pose_points_2.lifetime == rospy.Duration()

    def position_marker_3(self):
        self.pose_points_3 = Marker()
        self.pose_points_3.header.frame_id = 'map'
        self.pose_points_3.header.stamp=rospy.Time.now()
        self.pose_points_3.ns = "points"
        self.pose_points_3.id = 3
        self.pose_points_3.type = Marker.POINTS
        self.pose_points_3.action = Marker.ADD
        self.pose_points_3.pose.orientation.w = 1.0
        self.pose_points_3.scale.x = 0.3
        self.pose_points_3.scale.y = 0.3
        self.pose_points_3.color = ColorRGBA(0, 0, 1, 1)
        self.pose_points_3.lifetime == rospy.Duration()

if __name__ == '__main__':
    _getposition = get_global_position()
    _getposition.get_position()