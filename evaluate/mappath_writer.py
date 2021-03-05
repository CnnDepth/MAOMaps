import sys
import rospy
from nav_msgs.msg import Path
import numpy as np
import transformations as tf

rospy.init_node('mappath_writer')

path = []
timestamps = []
path_filename = sys.argv[1]

def path_callback(msg):
    global path
    print('Received path of {} poses'.format(len(msg.poses)))
    path = []
    for pose in msg.poses:
        xyz = pose.pose.position
        quat = pose.pose.orientation
        x, y, z = xyz.x, xyz.y, xyz.z
        roll, pitch, yaw = tf.euler_from_quaternion([quat.w, quat.x, quat.y, quat.z], axes='sxyz')
        path.append([x, y, z, roll, pitch, yaw])
    if len(timestamps) < len(path):
        timestamps.append(msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs)
    else:
        timestamps[-1] = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
    assert len(timestamps) == len(path)

path_subscriber = rospy.Subscriber('/mapPath', Path, path_callback)

rospy.spin()

print('Saving result to file {}'.format(path_filename))
print('Path contains {} poses'.format(len(path)))
fout = open(path_filename, 'w')
for x, y, z, roll, pitch, yaw in path:
    print(x, y, z, roll, pitch, yaw, file=fout)
fout.close()
fout = open(path_filename.replace('.txt', '_timestamps.txt'), 'w')
for ts in timestamps:
    print(ts, file=fout)
fout.close()