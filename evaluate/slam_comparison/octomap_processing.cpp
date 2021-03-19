#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <octomap/OcTree.h>
#include <math.h>
#include <time.h>
#include <chrono>

using namespace octomap;

std::string metric;

point3d* get_nearest_point_on_ray(const OcTree& octree,
                                  const point3d &ray_start,
                                  const point3d &ray_end)
{
    KeyRay ray;
    octree.computeRayKeys(ray_start, ray_end, ray);
    for (auto it = ray.begin(); it != ray.end(); it++)
    {
        OcTreeNode* node_found = octree.search(*it);
        if ((node_found != nullptr) && (octree.isNodeOccupied(*node_found)))
        {
            point3d* coords = new point3d;
            *coords = octree.keyToCoord(*it);
            return coords;
        }
    }
    return nullptr;
}

double distance(const point3d &a, const point3d &b)
{
    point3d diff = a - b;
    return sqrt(diff.x() * diff.x() + diff.y() * diff.y() + diff.z() * diff.z());
}

void get_occupied_voxels(const OcTree& octree, std::vector<point3d> &coords, std::vector<OcTreeKey> &keys)
{
    for (auto it = octree.begin_tree(); it != octree.end_tree(); it++)
    {
        if ((it.isLeaf()) && (octree.isNodeOccupied(*it)))
        {
            point3d pt(it.getX(), it.getY(), it.getZ());
            coords.push_back(pt);
            keys.push_back(it.getKey());
        }
    }
}

double compute_metric(const std::vector<point3d> &gt_points,
                      const std::vector<point3d> &slam_points,
                      const std::string &metric)
{
    double sum_distance = 0, n_points = 0;
    for (int i = 0; i < gt_points.size(); i++)
    {
        sum_distance += distance(gt_points[i], slam_points[i]);
        n_points += 1;
    }
    //std::cout << sum_distance << ' ' << n_points << std::endl;
    return sum_distance / n_points;
}

double compare_pointclouds(const Pointcloud& gt_pointcloud,
                           const std::vector<pose6d>& gt_poses, 
                           const Pointcloud& slam_pointcloud, 
                           const std::vector<pose6d>& slam_poses,
                           const double resolution,
                           const std::string &pose_choice,
                           std::ofstream& output)
{
    OcTree gt_octomap(resolution);
    OcTree slam_octomap(resolution);
    gt_octomap.insertPointCloud(gt_pointcloud, point3d(0., 0., 0.));
    slam_octomap.insertPointCloud(slam_pointcloud, point3d(0, 0, 0));
    std::vector<point3d> gt_points, slam_points;
    std::vector<point3d> gt_octomap_coords, slam_octomap_coords;
    std::vector<OcTreeKey> gt_octomap_keys, slam_octomap_keys;
    get_occupied_voxels(gt_octomap, gt_octomap_coords, gt_octomap_keys);
    get_occupied_voxels(slam_octomap, slam_octomap_coords, slam_octomap_keys);
    std::cout << "GT octomap has " << gt_octomap_coords.size() << " occupied nodes" << std::endl;
    std::cout << "Slam octomap has " << slam_octomap_coords.size() << " occupied nodes" << std::endl;
    std::vector<int> visibility_moments;
    auto start_time = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < slam_octomap_coords.size(); i++)
    {
        if (i % 100 == 0)
        {
            std::cout << "i: " << i << std::endl;
            std::cout << "Current metric: " << compute_metric(gt_points, slam_points, metric) << std::endl;
        }
        if (i == 1000)
        {
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> thousand_iter_time = end_time - start_time;
            std::cout << "1000 iterations time: " << thousand_iter_time.count() << std::endl;
        }
        point3d coord = slam_octomap_coords[i];
        OcTreeKey key = slam_octomap_keys[i];
        visibility_moments.clear();
        for (int t = 0; t < slam_poses.size(); t++)
        {
            point3d position(slam_poses[t].x(), slam_poses[t].y(), slam_poses[t].z());
            point3d direction_in_camera_system = slam_poses[t].inv().transform(coord);
            direction_in_camera_system /= direction_in_camera_system.x();
            if ((direction_in_camera_system.x() > 0) && \
                (abs(direction_in_camera_system.y()) < 1) && \
                (abs(direction_in_camera_system.z()) < 0.75)) // the point is in camera FoV
            {
                point3d* first_on_ray = get_nearest_point_on_ray(slam_octomap, position, position + (coord - position) * 10);
                if ((first_on_ray != nullptr) && (distance(coord, *first_on_ray) < 0.5))
                {
                    //std::cout << "Point " << coord.x() << ' ' << coord.y() << ' ' << coord.z() << \
                                 " is visible from position " << position.x() << ' ' << position.y() << ' ' << position.z() << std::endl;
                    visibility_moments.push_back(t);
                }
            }
        }
        if (visibility_moments.size() == 0)
        {
            std::cout << "WARNING: point " << coord.x() << ' ' << coord.y() << ' ' << coord.z() << " is not visible from any position!" << std::endl;
        }
        else
        {
            if (pose_choice == "first")
            {
                int first_pose = visibility_moments[0];
                visibility_moments.clear();
                visibility_moments.push_back(first_pose);
            }
            if (pose_choice == "last")
            {
                int last_pose = visibility_moments[visibility_moments.size() - 1];
                visibility_moments.clear();
                visibility_moments.push_back(last_pose);
            }
            if (pose_choice == "nearest")
            {
                double min_dist = 1e9;
                int best_t = 0;
                for (int i = 0; i < visibility_moments.size(); i++)
                {
                    int t = visibility_moments[i];
                    double cur_dist = distance(slam_poses[t].trans(), coord);
                    if (cur_dist < min_dist)
                    {
                        min_dist = cur_dist;
                        best_t = t;
                    }
                }
                visibility_moments.clear();
                visibility_moments.push_back(best_t);
            }
            if (pose_choice == "all")
            { }
            bool has_corresp = false;
            for (int ii = 0; ii < visibility_moments.size(); ii++)
            {
                int t = visibility_moments[ii];
                point3d direction_in_camera_system = slam_poses[t].inv().transform(coord);
                point3d gt_position = gt_poses[t].trans();
                //std::cout << "gt position: " << gt_position.x() << ' ' << gt_position.y() << ' ' << gt_position.z() << std::endl;
                //std::cout << "gt rotation: " << gt_poses[t].yaw() << std::endl;
                point3d gt_direction = gt_poses[t].transform(direction_in_camera_system) - gt_position;
                //std::cout << "slam direction: " << direction_in_camera_system.x() << ' ' << direction_in_camera_system.y() << ' ' << direction_in_camera_system.z() << std::endl;
                //std::cout << "gt direction: " << gt_direction.x() << ' ' << gt_direction.y() << ' ' << gt_direction.z() << std::endl;
                point3d* corresp_point = get_nearest_point_on_ray(gt_octomap, gt_poses[t].trans(), gt_position + gt_direction * 100);
                if (corresp_point == nullptr)
                {
                    //std::cout << "WARNING: point " << coord.x() << ' ' << coord.y() << ' ' << coord.z() << " has no correspondence in groundtruth map!" << std::endl;
                    /*std::cout << "slam direction: " << direction_in_camera_system.x() << ' ' << direction_in_camera_system.y() << ' ' << direction_in_camera_system.z() << std::endl;
                    std::cout << "gt direction: " << gt_direction.x() << ' ' << gt_direction.y() << ' ' << gt_direction.z() << std::endl;
                    std::cout << "gt position: " << gt_position.x() << ' ' << gt_position.y() << ' ' << gt_position.z() << std::endl;
                    std::cout << "gt rotation: " << gt_poses[t].yaw() << std::endl << std::endl;*/
                }
                else
                {
                    has_corresp = true;
                    /*std::cout << "Visibility moment: " << t << std::endl;
                    std::cout << "Slam point: " << coord.x() << ' ' << coord.y() << ' ' << coord.z() << std::endl;
                    std::cout << "Reference point: " << corresp_point->x() << ' ' << corresp_point->y() << ' ' << corresp_point->z() << std::endl << std::endl;
                    std::cout << "slam direction: " << direction_in_camera_system.x() << ' ' << direction_in_camera_system.y() << ' ' << direction_in_camera_system.z() << std::endl;
                    std::cout << "slam position: " << slam_poses[t].x() << ' ' << slam_poses[t].y() << ' ' << slam_poses[t].z() << std::endl;
                    std::cout << "gt direction: " << gt_direction.x() << ' ' << gt_direction.y() << ' ' << gt_direction.z() << std::endl;
                    std::cout << "gt position: " << gt_position.x() << ' ' << gt_position.y() << ' ' << gt_position.z() << std::endl;
                    std::cout << "gt rotation: " << gt_poses[t].yaw() << std::endl << std::endl;*/
                    if (metric == "abs")
                    {
                        gt_points.push_back(*corresp_point);
                        slam_points.push_back(coord);
                    }
                    if (metric == "rel")
                    {
                        gt_points.push_back(gt_poses[t].inv().transform(*corresp_point));
                        slam_points.push_back(slam_poses[t].inv().transform(coord));
                    }
                }
            }
            if (!has_corresp)
            {
                std::cout << "WARNING: point " << coord.x() << ' ' << coord.y() << ' ' << coord.z() << " has no correspondence in groundtruth map!" << std::endl;
            }
        }
    }
    for (int i = 0; i < gt_points.size(); i++)
    {
        output << gt_points[i].x() << ' ' << gt_points[i].y() << ' ' << gt_points[i].z() << ' ';
        output << slam_points[i].x() << ' ' << slam_points[i].y() << ' ' << slam_points[i].z() << std::endl;
    }
    return compute_metric(gt_points, slam_points, metric);
}

int main(int argc, char* argv[]) {
    std::ifstream gt_points_file(std::string(argv[1]) + "/gt_points.txt");
    std::ifstream gt_poses_file(std::string(argv[1]) + "/gt_poses.txt");
    std::ifstream slam_points_file(std::string(argv[1]) + "/slam_points.txt");
    std::ifstream slam_poses_file(std::string(argv[1]) + "/slam_poses.txt");
    std::cout << std::string(argv[1]) + "/slam_poses.txt" << std::endl;
    metric = std::string(argv[2]);
    std::string pose_choice = std::string(argv[3]);
    std::ofstream metric_output(argv[4]);
    std::cout << argv[4] << std::endl;
    float x, y, z, roll, pitch, yaw;
    Pointcloud gt_points, slam_points;
    std::vector<pose6d> gt_poses;
    std::vector<pose6d> slam_poses;
    while (gt_points_file >> x >> y >> z)
        gt_points.push_back(point3d(x, y, z));
    while (slam_points_file >> x >> y >> z)
        slam_points.push_back(point3d(x, y, z));
    std::cout << "GT and slam pointcloud sizes: " << gt_points.size() << ' ' << slam_points.size() << std::endl;
    while (gt_poses_file >> x >> y >> z >> roll >> pitch >> yaw)
        gt_poses.push_back(pose6d(x, y, z, roll, pitch, yaw));
    while (slam_poses_file >> x >> y >> z >> roll >> pitch >> yaw)
        slam_poses.push_back(pose6d(x, y, z, roll, pitch, yaw));
    std::cout << "GT and slam poses sizes: " << gt_poses.size() << ' ' << slam_poses.size() << std::endl;
    double metric_value = compare_pointclouds(gt_points,
                                              gt_poses, 
                                              slam_points, 
                                              slam_poses, 
                                              0.05, 
                                              pose_choice,
                                              metric_output);
    std::cout << "Mean distance: " << metric_value << std::endl;
    gt_points_file.close();
    gt_poses_file.close();
    slam_points_file.close();
    slam_poses_file.close();
    metric_output.close();
    return 0;
}