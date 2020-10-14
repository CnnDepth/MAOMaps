**Link to data**

The dataset is available [here](https://drive.google.com/drive/folders/1K88CglO9go3K4pJn_YUZCuWnT8yALefI).


**Dataset structure**

The dataset contains 20 pairs of trajectories. Each pair is stored in its own subdirectory.

Each sample contains:

* `first.bag`, `second.bag` - raw data in Rosbag format for first and second trajectory of the pair. Rosbags contain RGB images (in topic `/habitat/rgb/image`), depth maps (in topic `/habitat/depth/image`), camera info (in topic `/habitat/rgb/camera_info`), and poses (in topic `/true_pose`).

* `gt_points_first.pcd`, `gt_points_second.pcd` - ground truth maps stored as point clouds, for first and second trajectory of the pair.

* `gt_points_merged.pcd` - merged ground truth map in Pointcloud format.

* `gt_colors_first.txt`, `gt_colors_second.txt` - colors of points of first and second ground truth map in RGB format.

* `gt_colors_merged.txt` - colors of points of merged ground truth map in RGB format.

**Tools for evaluation**

The toolbox will be available here soon.
