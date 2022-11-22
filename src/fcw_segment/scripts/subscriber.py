#!/usr/bin/env python3

import rospy
import std_msgs.msg
import sensor_msgs.msg
import numpy as n
import ros_numpy

import open3d
import ros_open3d

pub = None

def pointcloud_callback(msg : sensor_msgs.msg.PointCloud2):
    arr = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

    cloud = ros_open3d.convertCloudFromRosToOpen3d(msg)

    rospy.loginfo("Received message")

    ros_msg = ros_open3d.convertCloudFromOpen3dToRos(cloud)
    pub.publish(ros_msg)

    open3d.io.write_point_cloud("pointcloud_frame.pcd", cloud)

def main():
    global pub

    rospy.init_node('listener', anonymous=True)

    pub = rospy.Publisher("/filtered_points", sensor_msgs.msg.PointCloud2)

    sub = rospy.Subscriber("/rslidar_points", sensor_msgs.msg.PointCloud2, pointcloud_callback)

    rospy.spin()

if __name__ == "__main__":
    main()
