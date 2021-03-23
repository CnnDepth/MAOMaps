import sys
import transformations as tf
import numpy as np


input_filename = sys.argv[1]
if input_filename.endswith('.txt'):
    output_filename = input_filename.replace('.txt', '_transformed.txt')
elif input_filename.endswith('.pcd'):
    output_filename = input_filename.replace('.pcd', '_transformed.pcd')
start_pose_filename = sys.argv[2]
correction_angle = float(sys.argv[3])

input_file = open(input_filename, 'r')
lines = input_file.readlines()
cap = []
input_pcd = []
for line in lines:
    try:
        input_pcd.append(list(map(float, line.split())))
    except ValueError:
        cap.append(line)
input_file.close()
input_pcd = np.array(input_pcd)

pose_file = open(start_pose_filename, 'r')
position = np.array(list(map(float, pose_file.readline().split())))
z_angle = float(pose_file.readline().strip()) + np.pi + correction_angle
print(position, z_angle)
pose_file.close()

output_pcd = input_pcd.copy()
output_pcd[:, 0] = input_pcd[:, 0] * np.cos(z_angle) - input_pcd[:, 1] * np.sin(z_angle)
output_pcd[:, 1] = input_pcd[:, 0] * np.sin(z_angle) + input_pcd[:, 1] * np.cos(z_angle)
output_pcd += np.array(position)

output_file = open(output_filename, 'w')
for line in cap:
    output_file.write(line)
for pt in output_pcd:
    print(' '.join(list(map(str, pt))), file=output_file)
output_file.close()
print('Pointcloud transformed and saved')