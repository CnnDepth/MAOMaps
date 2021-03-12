## Dataset description

MAOMaps is a dataset for evaluation of Visual SLAM, RGB-D SLAM and Map Merging algorithms. It contains 40 samples with RGB and depth images, and ground truth trajectories and maps. These 40 samples are joined into 20 pairs of overlapping maps for map merging methods evaluation. The samples were collected using [Matterport3D](https://niessner.github.io/Matterport/) dataset and [Habitat](https://aihabitat.org/) simulator.

![Image](img/maomaps_screen.png?raw=true "Title")

### Link to data

The dataset is available [here](https://drive.google.com/drive/folders/1K88CglO9go3K4pJn_YUZCuWnT8yALefI).


### Dataset structure

The dataset contains 20 pairs of trajectories. Each pair is stored in its own subdirectory.

Each sample contains:

* `first.bag`, `second.bag` - raw data in Rosbag format for first and second trajectory of the pair. Rosbags contain RGB images (in topic `/habitat/rgb/image`), depth maps (in topic `/habitat/depth/image`), camera info (in topic `/habitat/rgb/camera_info`), and poses (in topic `/true_pose`).

* `gt_points_first.pcd`, `gt_points_second.pcd` - ground truth maps stored as point clouds, for first and second trajectory of the pair.

* `gt_points_merged.pcd` - merged ground truth map in Pointcloud format.

* `gt_colors_first.txt`, `gt_colors_second.txt` - colors of points of first and second ground truth map in RGB format.

* `gt_colors_merged.txt` - colors of points of merged ground truth map in RGB format.

## Toolbox

TODO
