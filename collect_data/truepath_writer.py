import sys
import rospy
import transformations as tf
from geometry_msgs.msg import PoseStamped
import numpy as np

path = []
truepath_filename = sys.argv[1]
rospy.init_node('true_path_writer')

def pose_callback(msg):
    path.append([msg.pose.position.x, msg.pose.position.y])

pose_subscriber = rospy.Subscriber('true_pose', PoseStamped, pose_callback)

rospy.spin()

print('Path containts {} poses'.format(len(path)))
print('Saving path to', truepath_filename)
np.savetxt(truepath_filename, np.array(path))