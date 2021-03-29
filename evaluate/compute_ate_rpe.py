import numpy as np


def projection(pcd, resolution=0.05, max_obstacle_height=1.5):
    pcd = np.round(pcd / resolution) * resolution
    max_npoints = 0
    ground_z = None
    zs = np.unique(pcd[:, 2])
    for z in zs:
        if z > 0.5 * (zs.min() + zs.max()):
            continue
        n_points = len(pcd[pcd[:, 2] == z])
        if n_points > max_npoints:
            max_npoints = n_points
            ground_z = z
    
    ground_points = pcd[(pcd[:, 2] >= ground_z - 0.1) * (pcd[:, 2] <= ground_z + 0.1)]
    obst_points = pcd[(pcd[:, 2] > ground_z + 0.1) * (pcd[:, 2] < ground_z + max_obstacle_height)]
    ground_proj = np.round(ground_points[:, :2] / resolution).astype(np.int32)
    ground_proj = np.array(list(set(map(tuple, ground_proj))))
    obst_proj = np.round(obst_points[:, :2] / resolution).astype(np.int32)
    obst_proj = np.array(list(set(map(tuple, obst_proj))))
    
    min_coord = np.minimum(ground_proj.min(axis=0), obst_proj.min(axis=0))
    max_coord = np.maximum(ground_proj.max(axis=0), obst_proj.max(axis=0))
    h, w = max_coord - min_coord + 1
    
    gt_projmap = np.ones((w, h), dtype=np.uint8) * 127
    ground_proj -= min_coord
    obst_proj -= min_coord
    gt_projmap[ground_proj[:, 1], ground_proj[:, 0]] = 255
    gt_projmap[obst_proj[:, 1], obst_proj[:, 0]] = 0
    return gt_projmap, min_coord


def rotate(points, angle):
    res = points.copy()
    res[:, 0] = points[:, 0] * np.cos(angle) + points[:, 1] * np.sin(angle)
    res[:, 1] = -points[:, 0] * np.sin(angle) + points[:, 1] * np.cos(angle)
    return res


def align_points(slam_map, gt_map, x, y, angle, map_x, map_y, shift, scale):
    points = np.array((slam_map == 0).nonzero()).T.astype(np.float32)
    points[:, 0] += map_y * 20
    points[:, 1] += map_x * 20
    points_rotated = rotate(points, angle)
    points_transformed = points_rotated / scale
    points_transformed[:, 0] += -shift[1] + y * 20
    points_transformed[:, 1] += -shift[0] + x * 20
    gt_points = np.array((gt_map == 0).nonzero()).T.astype(np.float32)
    return points_transformed, gt_points


def compare(slam_map, gt_map, x, y, angle, map_x, map_y, shift, scale=1):
    slam_points, gt_points = align_points(slam_map, gt_map, x, y, angle, map_x, map_y, shift, scale)
    sum_dst = 0
    for pt in slam_points:
        dst = np.sqrt(np.sum((gt_points - pt) ** 2, axis=1)).min()
        sum_dst += dst
    return sum_dst * 0.05 / len(slam_points)


def compare_points(slam_points, gt_points):
    sum_dst = 0
    for pt in slam_points:
        dst = np.sqrt(np.sum((gt_points - pt) ** 2, axis=1)).min()
        sum_dst += dst
    return sum_dst * 0.05 / len(slam_points)