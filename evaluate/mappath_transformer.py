import sys
import transformations as tf
import numpy as np

input_filename = sys.argv[1]
output_filename = input_filename.replace('.txt', '_transformed.txt')
pose_filename = sys.argv[2]
correction_angle = float(sys.argv[3])
print('Transforming path', input_filename)

pose_file = open(pose_filename, 'r')
start_position = np.array(list(map(float, pose_file.readline().split())))
start_angle = float(pose_file.readline().strip()) + np.pi + correction_angle
#start_angle = -start_angle
pose_file.close()

path = np.loadtxt(input_filename)
path_transformed = path.copy()
path_transformed[:, 0] = path[:, 0] * np.cos(start_angle) - path[:, 1] * np.sin(start_angle)
path_transformed[:, 1] = path[:, 0] * np.sin(start_angle) + path[:, 1] * np.cos(start_angle)
path_transformed[:, :3] += np.array(start_position)
path_transformed[:, -1] += start_angle

np.savetxt(output_filename, path_transformed)
print('Path transformed and saved')