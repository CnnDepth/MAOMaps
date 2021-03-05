import sys
import transformations as tf
import numpy as np

correction_angle = [-0.45,
 -0.45,
 -0.45,
 -0.45,
 1.51,
 1.51,
 1.51,
 1.51,
 -0.37,
 -0.37,
 -0.37,
 0.06,
 0.07,
 0.07,
 0.07,
 0.07,
 0.07,
 0.07,
 0.07,
 0.07]

for i in range(18, 21):
    for map_name in ['first', 'second']:
        input_filename = 'sample{}/deepvo_poses_{}.txt'.format(i, map_name)
        output_filename = 'sample{}/deepvo_poses_{}_transformed.txt'.format(i, map_name)
        pose_filename = 'sample{}/start_pose_{}.txt'.format(i, map_name)
        print('Transforming path', input_filename)

        pose_file = open(pose_filename, 'r')
        start_position = np.array(list(map(float, pose_file.readline().split())))
        start_angle = float(pose_file.readline().strip()) + np.pi + correction_angle[i - 1]
        #start_angle = -start_angle
        print(start_angle)
        pose_file.close()

        path = np.loadtxt(input_filename)
        path_transformed = path.copy()
        path_transformed[:, 0] = path[:, 0] * np.cos(start_angle) - path[:, 1] * np.sin(start_angle)
        path_transformed[:, 1] = path[:, 0] * np.sin(start_angle) + path[:, 1] * np.cos(start_angle)
        path_transformed[:, :3] += np.array(start_position)
        path_transformed[:, -1] += start_angle

        np.savetxt(output_filename, path_transformed)
        print('Path transformed and saved')