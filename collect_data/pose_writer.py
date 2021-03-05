import sys
import rospy
import transformations as tf
from geometry_msgs.msg import PoseStamped
import numpy as np

start_pose = None
rospy.init_node('pose_pooper')

def pose_callback(data):
    global start_pose
    if start_pose is not None:
        return
    print('Start pose received')
    start_pose = data.pose
    print('position:', start_pose.position.x, start_pose.position.y, start_pose.position.z)
    print('orientation:', start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w)

pose_subscriber = rospy.Subscriber('true_pose', PoseStamped, pose_callback)

rospy.spin()

_, __, z_angle = tf.euler_from_quaternion([start_pose.orientation.w, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z], axes='sxyz')
print('Rotation angle:', z_angle)
fout = open(sys.argv[1], 'w')
print(start_pose.position.x, start_pose.position.y, start_pose.position.z, file=fout)
print(z_angle, file=fout)
fout.close()