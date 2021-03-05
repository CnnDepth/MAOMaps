import rospy
import struct
import numpy as np
import transformations as tf
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from skimage.io import imsave
from cv_bridge import CvBridge
import sys

points = {}
poses = {}
rgbs = {}
depths = {}
bridge = CvBridge()

rospy.init_node('gt_map_creator')
points_filename = sys.argv[1]
colors_filename = sys.argv[2]
add_angle = float(sys.argv[3])

def get_local_pointcloud(rgb, depth, fov=90):
    fov = fov / (180 / np.pi)
    H, W, _ = rgb.shape
    idx_h = np.tile(np.arange(H), W).reshape((W, H)).T.astype(np.float32) - 120
    idx_w = np.tile(np.arange(W), H).reshape((H, W)).astype(np.float32) - 160
    idx_h /= (W / 2 * np.tan(fov / 2))
    idx_w /= (W / 2 * np.tan(fov / 2))
    points = np.array([np.ones((H, W)), -idx_w, -idx_h])
    points = np.transpose(points, [1, 2, 0])
    points_dist = np.sqrt(np.sum(points ** 2, axis=2))
    points = points * depth[:, :, np.newaxis]
    points = np.array([points[:, :, 0].ravel(), points[:, :, 1].ravel(), points[:, :, 2].ravel()]).T
    return points

def rgb_callback(msg):
    rgbs[msg.header.stamp] = np.array(bridge.imgmsg_to_cv2(msg))

def depth_callback(msg):
    img = np.array(bridge.imgmsg_to_cv2(msg))
    depths[msg.header.stamp] = np.array(bridge.imgmsg_to_cv2(msg))

def pose_callback(msg):
    print('X Y Z', msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    print('Orientation:', msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
    poses[msg.header.stamp] = msg.pose

rgb_subscriber = rospy.Subscriber('habitat/rgb/image', Image, rgb_callback)
depth_subscriber = rospy.Subscriber('habitat/depth/image', Image, depth_callback)
pose_subscriber = rospy.Subscriber('true_pose', PoseStamped, pose_callback)

rospy.spin()

"""
positions = []
orientations = []
pointclouds = []
rgbs_total = []
depths_total = []
for stamp in sorted(rgbs.keys()):
    try:
        position = poses[stamp].position
        orientation = poses[stamp].orientation
        points = get_local_pointcloud(rgbs[stamp], depths[stamp])
        rgb = rgbs[stamp]
        depth = depths[stamp]
    except:
        continue
    positions.append([position.x, position.y, position.z])
    orientations.append([orientation.x, orientation.y, orientation.z, orientation.w])
    pointclouds.append(points)
    rgbs_total.append(rgb)
    depths_total.append(depth)
np.savetxt('positions.txt', positions)
np.savetxt('orientations.txt', orientations)
np.savez('pointclouds.npz', pointclouds)
np.savez('depths.npz', depths_total)
np.savez('rgbs.npz', rgbs_total)
"""

print('Begin collecting pointcloud...')
point_colors = {}
counter = {}
for stamp in rgbs.keys():
    try:
        position = poses[stamp].position
        orientation = poses[stamp].orientation
        _, __, z_angle = tf.euler_from_quaternion([orientation.w, orientation.x, orientation.y, orientation.z], axes='sxyz')
        z_angle += np.pi + add_angle
        points = get_local_pointcloud(rgbs[stamp], depths[stamp])
    except KeyError:
        continue
    points_rotated = points.copy()
    points_rotated[:, 0] = points[:, 0] * np.cos(z_angle) - points[:, 1] * np.sin(z_angle)
    points_rotated[:, 1] = points[:, 0] * np.sin(z_angle) + points[:, 1] * np.cos(z_angle)
    x, y, z = position.x, position.y, position.z
    points_transformed = points_rotated + np.array([x, y, z])
    points_dst = np.sqrt(points[:, 0] ** 2 + points[:, 1] ** 2 + points[:, 2] ** 2)
    good_point_ids = (points_dst > 0.01) * (points_dst < 7)
    points_rounded = np.round(points_transformed[good_point_ids] / 0.05).astype(np.int16)
    colors = rgbs[stamp].reshape((240 * 320, 3))[good_point_ids]
    for pt, color in zip(points_rounded, colors):
        pt_tuple = tuple(pt)
        point_colors[pt_tuple] = color
        counter[pt_tuple] = counter.get(pt_tuple, 0) + 1
print('Collected')
print('Cloud containts {} points'.format(len(point_colors)))

print("Saving result...")
points_output = open(points_filename, 'w')
rgbs_output = open(colors_filename, 'w')
for pt in point_colors.keys():
    cnt = counter[pt]
    #if cnt < 3:
    #    continue
    pt_float = np.array(pt).astype(np.float32) * 0.05
    points_output.write(' '.join([str(x) for x in pt_float]) + '\n')
    rgb = point_colors[pt]
    rgbs_output.write(' '.join([str(x) for x in rgb]) + '\n')
points_output.close()
rgbs_output.close()
print("Saved")
