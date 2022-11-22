#!/usr/bin/python3

import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
import time

def segment_and_color(cloud):
    labels = np.array(cloud.cluster_dbscan(eps=0.3, min_points=15))

    max_label = labels.max()
    
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))

    colors[labels < 0] = 0
    
    cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

    return cloud

def downsample(cloud, sample_diff = 0.1):
    downpcd = cloud.voxel_down_sample(sample_diff)
    return downpcd

if __name__ == "__main__":
    print("Running")
    pcd = o3d.io.read_point_cloud("../data/kitti.pcd")
    print(pcd)
    # o3d.visualization.draw_geometries([pcd])

    tick = time.time()

    cloud = downsample(pcd)

    result = segment_and_color(cloud)

    tock = time.time()
    print("Took %f seconds to process"%(tock-tick))

    o3d.visualization.draw_geometries([pcd])
    o3d.visualization.draw_geometries([cloud])
    o3d.visualization.draw_geometries([result])
