#!/usr/bin/env python3

import rospy
import std_msgs.msg
import sensor_msgs.msg

import numpy as np
import ros_numpy

import matplotlib.pyplot as plt

import open3d as o3d

import ros_open3d

import visualization_msgs.msg

import time

segment_pub     = None
inlier_pub      = None
sample_pub      = None

def segment_and_color(cloud):
    labels = np.array(cloud.cluster_dbscan(eps=0.3, min_points=15))

    max_label = labels.max()
    
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))

    colors[labels < 0] = 0
    
    cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

    return cloud

def downsample(cloud):
    downpcd = cloud.voxel_down_sample(0.1)
    return downpcd

def detect_plane(cloud):
    plane_model, inliers    = cloud.segment_plane(distance_threshold=0.5, ransac_n=3, num_iterations=100)
    inlier_cloud            = cloud.select_by_index(inliers)
    outlier_cloud           = cloud.select_by_index(inliers, invert=True)

    return (inlier_cloud, outlier_cloud)
    
def pointcloud_callback(msg : sensor_msgs.msg.PointCloud2):
    arr   = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

    cloud = ros_open3d.convertCloudFromRosToOpen3d(msg)

    rospy.logdebug("Received pointcloud message")

    tick = time.time()

    cloud = downsample(cloud)

    rospy.logdebug(" Downsampling    : %.3f s"%(time.time() - tick))
    tick = time.time()

    inliers, outliers = detect_plane(cloud)

    rospy.logdebug(" RANSAC          : %.3f s"%(time.time() - tick))
    tick = time.time()

    result = segment_and_color(outliers)

    rospy.logdebug(" Segmentation    : %.3f s"%(time.time() - tick))

    seg_msg = ros_open3d.convertCloudFromOpen3dToRos(result)
    seg_msg.header.frame_id = msg.header.frame_id

    plane_msg = ros_open3d.convertCloudFromOpen3dToRos(inliers)
    plane_msg.header.frame_id = msg.header.frame_id

    sample_msg = ros_open3d.convertCloudFromOpen3dToRos(cloud)
    sample_msg.header.frame_id = msg.header.frame_id

    segment_pub.publish(seg_msg)
    inlier_pub.publish(plane_msg)
    sample_pub.publish(sample_msg)

def main():
    global segment_pub, inlier_pub, sample_pub

    rospy.init_node('listener', anonymous=True, log_level=rospy.DEBUG)

    sample_pub  = rospy.Publisher("/sampled_points", sensor_msgs.msg.PointCloud2, queue_size=10)

    segment_pub = rospy.Publisher("/segment_points", sensor_msgs.msg.PointCloud2, queue_size=10)

    inlier_pub  = rospy.Publisher("/plane_ponts", sensor_msgs.msg.PointCloud2, queue_size=10)

    sub = rospy.Subscriber("/rslidar_points", sensor_msgs.msg.PointCloud2, pointcloud_callback, queue_size=10)

    rospy.spin()

if __name__ == "__main__":
    main()
