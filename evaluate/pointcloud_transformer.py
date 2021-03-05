import sys
import transformations as tf
import numpy as np

best_scales = {
'sample1/first': 1.18,
 'sample1/second': 1.11,
 'sample2/first': 1.18,
 'sample2/second': 1.18,
 'sample3/first': 1.43,
 'sample3/second': 1.39,
 'sample4/first': 1.11,
 'sample4/second': 1.0,
 'sample5/first': 1.33,
 'sample5/second': 1.43,
 'sample6/first': 1.33,
 'sample6/second': 1.43,
 'sample7/first': 1.33,
 'sample7/second': 1.43,
 'sample8/first': 1.32,
 'sample8/second': 1.39,
 'sample9/first': 1.43,
 'sample9/second': 1.59,
 'sample10/first': 1.05,
 'sample10/second': 1.33,
 'sample11/first': 1.59,
 'sample11/second': 1.37,
'sample12/first': 1.05,
 'sample12/second': 1.2,
 'sample13/first': 1.2,
 'sample13/second': 1.2,
 'sample14/first': 1.15,
 'sample14/second': 1.05,
 'sample15/first': 1.2,
 'sample15/second': 1.45,
 'sample16/first': 1.8,
 'sample16/second': 1.3,
 'sample17/first': 1.55,
 'sample17/second': 1.75,
 'sample18/first': 1.6,
 'sample18/second': 1.55,
 'sample19/first': 1.55,
 'sample19/second': 1.5,
 'sample20/first': 1.45,
 'sample20/second': 1.8}

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

for i in range(12, 21):
    for map_name in ['first', 'second']:
        input_filename = 'sample{}/fcnn_points_{}_warped.pcd'.format(i, map_name)
        output_filename = 'sample{}/fcnn_points_{}_warped_transformed.pcd'.format(i, map_name)
        pose_filename = 'sample{}/start_pose_{}.txt'.format(i, map_name)
        print('Transforming pointcloud', input_filename)

        try:
            input_file = open(input_filename, 'r')
        except:
            continue
        print(input_filename)
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

        pose_file = open(pose_filename, 'r')
        position = np.array(list(map(float, pose_file.readline().split())))
        #rotation = list(map(float, pose_file.readline().split()))
        #x, y, z, w = rotation
        #_, __, z_angle = tf.euler_from_quaternion([w, x, y, z], axis='sxyz')
        z_angle = float(pose_file.readline().strip()) + np.pi + correction_angle[i - 1]
        print(position, z_angle)
        pose_file.close()

        output_pcd = input_pcd.copy()
        output_pcd[:, 0] = input_pcd[:, 0] * np.cos(z_angle) - input_pcd[:, 1] * np.sin(z_angle)
        output_pcd[:, 1] = input_pcd[:, 0] * np.sin(z_angle) + input_pcd[:, 1] * np.cos(z_angle)
        #output_pcd /= best_scales['sample{}/{}'.format(i, map_name)]
        output_pcd += np.array(position)

        output_file = open(output_filename, 'w')
        for line in cap:
            output_file.write(line)
        for pt in output_pcd:
            print(' '.join(list(map(str, pt))), file=output_file)
        output_file.close()
        print('Pointcloud transformed and saved')