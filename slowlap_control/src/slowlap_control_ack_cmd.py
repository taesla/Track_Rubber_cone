#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Float32
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from matplotlib import pyplot as plt
from geometry_msgs.msg import Twist


def callback(msg):
    
    blue_x = []
    blue_y = []

    yellow_x = []
    yellow_y = []

    mid_path_points_x = []
    mid_path_points_y = []

    for p in pc2.read_points(msg , field_names = ("x","y","z","rgb"), skip_nans = True):

        if p[3] == 4278190335 :
            blue_x.append(p[0])
            blue_y.append(p[1])

        else :
            yellow_x.append(p[0])
            yellow_y.append(p[1])

    def sort_list(list1, list2):
 
        zipped_pairs = zip(list2, list1)
    
        z = [x for _, x in sorted(zipped_pairs)]
        
        return z
    
    sort_list(blue_y,blue_x)
    blue_x.sort()
    
    sort_list(yellow_y,yellow_x)
    yellow_x.sort()

    lane_list = Marker()

    lane_list.header.frame_id = 'velodyne'
    lane_list.type = Marker.LINE_LIST

    for i in range(len(blue_x)):
        for j in range(len(yellow_x)):
            lane_list.points.append(Point(blue_x[i],blue_y[i],0))
            lane_list.points.append(Point(yellow_x[j],yellow_y[j],0))
    lane_list.color = ColorRGBA(0, 1, 1, 1)
    lane_list.scale.x = 0.05

    lane_pub.publish(lane_list)

    line_list = Marker()

    line_list.header.frame_id = 'velodyne'
    line_list.type = Marker.LINE_STRIP
    line_list.action = Marker.ADD
    path = Twist()

    for i in range(len(blue_x)):
        for j in range(len(yellow_x)):

            mid_path_points_x.append(( blue_x[i] + yellow_x[j] ) / 2)
            mid_path_points_y.append(( blue_y[i] + yellow_y[j] ) / 2)
            sort_list(mid_path_points_y,mid_path_points_x)
            mid_path_points_x.sort()
    path.linear.x = mid_path_points_x[0]
    path.linear.y = mid_path_points_y[0]

    x=np.linspace(3,8,100)
    mid_path_points_x.insert(0,3)
    mid_path_points_y.insert(0,0)
    mid_path_points_x.insert(0,2)
    mid_path_points_y.insert(0,0)
    mid_path_points_x.insert(0,1)
    mid_path_points_y.insert(0,0)
    mid_path_points_x.insert(0,0)
    mid_path_points_y.insert(0,0)
    p = np.polyfit(mid_path_points_x,mid_path_points_y,2)
    path_func = p[0] * x ** 2 + p[1] * x + p[2]
 
    
    path.angular.z = np.mean(np.gradient(path_func))
   
        

    path_pub.publish(path)


if __name__=='__main__':
    rospy.init_node('slowlap_control')
    rospy.Subscriber("sorted_points2",PointCloud2,callback)
    line_pub = rospy.Publisher("cone_line",Marker,queue_size=1)
    lane_pub = rospy.Publisher("cone_lane",Marker,queue_size=1)
    path_pub = rospy.Publisher("cmd_vel",Twist,queue_size=1)
    steer_pub = rospy.Publisher("assist_steer",Float32,queue_size=1)

    rospy.spin()