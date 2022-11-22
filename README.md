# LiDAR-Segmentation

 - Segmentation of point cloud data from LiDAR. 
 - The data is from udacity. I have used DBSCAN&RANSAC clustering, Voxel grid down sampling in the pipeline. 
 - There's a ![script](./src/fcw_segment/scripts/ros_open3d.py) for converting point cloud format between Open3D and ROS:   
    * convertCloudFromOpen3dToRos  
    * convertCloudFromRosToOpen3d


## Rviz Topic output:

### /plane_points
![](./docd/plane_points.png)

### /rslidar_points
![](./docd/rslidar_points.png)

### /sampled_points
![](./docd/sampled_points.png)

### /segment_points
![](./docd/segment_points.png)

## Setup Locally:

 - Enter to project directory

  `cd LiDAR-Segmentation`

 - Source the workspace

  `source devel/setup.bash`

 - Run the launch files

  `roslaunch fcw_segment frame_pub.launch`

  `roslaunch fcw_segment main.launch`
  
 Nb:- If you Still got an error then remove `build` and `devel` folders and run the command `catkin_make`. Now follow the above steps
