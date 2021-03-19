# slam_comparison

This code implements the metrics for vSLAM map (pointcloud) quality estimation as in our SibCon 2021 paper. If you use this code in your research, please cite this paper: TBA.

## Pre-requesites

1. Robot operating system (ROS) >= Kinetic
2. PCL Library > 1.8
3. Octomap package

## Compile

1. Install the pre-requesites

```bash
TBA
```

2. Clone the package into your catkin workspace and make the package

```bash
roscd && cd src
git clone https://github.com/CnnDepth/slam_comparison.git
cd .. && catkin_make
```

## Example usage

### 1. Subscriber node

**pointcloud_subscriber** - the node that subscribes to PointCloud topic, receives pointcloud from it, and saves this pointcloud to file

**Usage**

1. Run

`rosrun pointcloud processing pointcloud_subscriber <pointcloud topic> <coordinates file> <colors file>`

2. When pointcloud is received, press Ctrl-C to interrupt subscriber and save the pointcloud.

**Example**

```
rosbag play sample_from_robot.bag
roslaunch rtabmap_ros rtabmap.launch
rosrun pointcloud_processing pointcloud_subscriber /octomap_occupied_space ./pcd_data/points.txt ./pcd_data/colors.txt
```

### 2. Publisher node

**pointcloud_publisher** - the node that publishes pointcloud which is stored on disk

**Usage**

`rosrun pointcloud_processing pointcloud_publisher <point_topic> <coordinates file> <colors file>`

**Example**

`rosrun pointcloud_processing pointcloud_publisher /cloud ./pcd_data/points.txt ./pcd_data/colors.txt`

### 3. Metric calculating node

**octomap_processing** - the node that takes ground truth and SLAM-builded pointcloud and trajectory, and computes our AME metric of them

**Usage**

1. Run

`rosrun pointcloud_processing octomap_processing <path_to_read_pcds> <metric> <pose_choice> <path_to_save_results>`

where

`path_to_read_pcds` - the directory with ground-truth and SLAM-builded pointcloud and trajectory. They must be stored as `gt_points.txt`, `gt_poses.txt`, `slam_points.txt`, `slam_poses.txt` respectively in this directory

`metric` - the kind of metric to compute, `abs` or `rel`

`pose_choice` - choice of visibility moment `t` for each point (see the paper). Can be `first`, `last`, `nearest` of `all`

`path_to_save_results` - the path where to store found correspondences between ground-truth and SLAM-builded points

2. Take a cup of coffee and wait for results. When node finishes, the correspondences will be stored at `path_to_save_results` file, and metric value will be written to console as `Mean distance: 0.735`

**Example**

`rosrun pointcloud_processing octomap_processing ./sample1/data abs nearest ./sample1/results.txt`
