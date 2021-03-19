import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped
from cv_bridge import CvBridge
import yaml
import transformations as tf
import numpy as np

MAX_DEPTH = 10


def getCameraInfo(filepath):
    with open(filepath, 'r') as f:
        yaml_data = yaml.safe_load(f)
    width = yaml_data['image_width']
    height = yaml_data['image_height']
    D = yaml_data['distortion_coefficients']['data']
    K = yaml_data['camera_matrix']['data']
    R = yaml_data['rectification_matrix']['data']
    P = yaml_data['projection_matrix']['data']
    return CameraInfo(width=width, height=height, D=D, K=K, R=R, P=P)


class HabitatObservationPublisher:

    def __init__(self,
                 rgb_topic=None,
                 depth_topic=None,
                 camera_info_topic=None,
                 true_pose_topic=None,
                 camera_info_file=None):
        self.cvbridge = CvBridge()

        # Initialize camera info publisher
        if camera_info_topic is not None:
            self.publish_camera_info = True
            self.camera_info_publisher = rospy.Publisher(camera_info_topic, CameraInfo, latch=True, queue_size=100)
            self.camera_info = getCameraInfo(camera_info_file)
        else:
            self.publish_camera_info = False

        # Initialize RGB image publisher
        if rgb_topic is not None:
            self.publish_rgb = True
            self.image_publisher = rospy.Publisher(rgb_topic, Image, latch=True, queue_size=100)
            self.image = Image()
            self.image.encoding='rgb8'
            self.image.is_bigendian = False
        else:
            self.publish_rgb = False

        # Initialize depth image publisher
        if depth_topic is not None:
            self.publish_depth = True
            self.depth_publisher = rospy.Publisher(depth_topic, Image, latch=True, queue_size=100)
            self.depth = Image()
            self.depth.encoding = 'mono8'
            self.depth.is_bigendian = True
        else:
            self.publish_depth = False

        # Initialize position publisher
        if true_pose_topic is not None:
            self.publish_true_pose = True
            self.pose_publisher = rospy.Publisher(true_pose_topic, PoseStamped, latch=True, queue_size=100)
        else:
            self.publish_true_pose = False


    def publish(self, observations):
        cur_time = rospy.Time.now()

        # Publish RGB image
        if self.publish_rgb:
            self.image = self.cvbridge.cv2_to_imgmsg(observations['rgb'])
            self.image.encoding = 'rgb8'
            self.image.header.stamp = cur_time
            self.image.header.frame_id = 'camera_link'
            self.image_publisher.publish(self.image)

        # Publish depth image
        if self.publish_depth:
            self.depth = self.cvbridge.cv2_to_imgmsg(observations['depth'] * MAX_DEPTH)
            self.depth.header.stamp = cur_time
            self.depth.header.frame_id = 'base_scan'
            self.depth_publisher.publish(self.depth)

        # Publish camera info
        if self.publish_camera_info:
            self.camera_info.header.stamp = cur_time
            self.camera_info_publisher.publish(self.camera_info)

        # Publish true pose
        if self.publish_true_pose:
            position, rotation = observations['agent_position']
            y, z, x = position
            cur_orientation = rotation
            cur_euler_angles = tf.euler_from_quaternion([cur_orientation.w, cur_orientation.x, cur_orientation.z, cur_orientation.y])
            cur_x_angle, cur_y_angle, cur_z_angle = cur_euler_angles
            cur_z_angle += np.pi
            cur_pose = PoseStamped()
            cur_pose.header.stamp = cur_time
            cur_pose.header.frame_id = 'map'
            cur_pose.pose.position.x = x
            cur_pose.pose.position.y = y
            cur_pose.pose.position.z = z
            cur_pose.pose.orientation.w, cur_pose.pose.orientation.x, cur_pose.pose.orientation.y, cur_pose.pose.orientation.z = tf.quaternion_from_euler(0, 0, cur_z_angle)
            self.pose_publisher.publish(cur_pose)