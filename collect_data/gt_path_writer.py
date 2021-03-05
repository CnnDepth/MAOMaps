import sys
import rospy
import transformations as tf
from geometry_msgs.msg import PoseStamped
import numpy as np

path = []
timestamps = []
truepath_filename = sys.argv[1]
rospy.init_node('true_path_writer')

def pose_callback(msg):
    xyz = msg.pose.position
    quat = msg.pose.orientation
    x, y, z = xyz.x, xyz.y, xyz.z
    roll, pitch, yaw = tf.euler_from_quaternion([quat.w, quat.x, quat.y, quat.z], axes='sxyz')
    path.append([x, y, z, roll, pitch, yaw])
    timestamps.append(msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs)

pose_subscriber = rospy.Subscriber('true_pose', PoseStamped, pose_callback)

rospy.spin()

print('Path contains {} poses'.format(len(path)))
fout = open(truepath_filename, 'w')
for x, y, z, roll, pitch, yaw in path:
    print(x, y, z, roll, pitch, yaw, file=fout)
fout.close()

fout = open(truepath_filename.replace('.txt', '_timestamps.txt'), 'w')
for ts in timestamps:
    print(ts, file=fout)
fout.close()