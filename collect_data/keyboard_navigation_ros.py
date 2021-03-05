import habitat
from habitat.sims.habitat_simulator.actions import HabitatSimActions
import rospy
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped
from cv_bridge import CvBridge
import numpy as np
import keyboard
import argparse
import transformations as tf
from typing import Any
from gym import spaces
from habitat.utils.visualizations import maps
from skimage.io import imsave
from tqdm import tqdm
import h5py

#rate = 30
rospy.init_node('keyboard_navigation_ros')
rate = rospy.Rate(30)
D = [0, 0, 0, 0, 0]
K = [160, 0.0, 160.5, 0.0, 160, 120.5, 0.0, 0.0, 1.0]
R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
P = [160, 0.0, 160.5, 0.0, 0.0, 160, 120.5, 0.0, 0.0, 0.0, 1.0, 0.0]
MAX_DEPTH = 10

def inverse_transform(x, y, start_x, start_y, start_angle):
    new_x = (x - start_x) * np.cos(start_angle) + (y - start_y) * np.sin(start_angle)
    new_y = -(x - start_x) * np.sin(start_angle) + (y - start_y) * np.cos(start_angle)
    return new_x, new_y

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
    #points = points / points_dist[:, :, np.newaxis] * depth * 10.0
    points = points * depth * MAX_DEPTH
    points = np.array([points[:, :, 0].ravel(), points[:, :, 1].ravel(), points[:, :, 2].ravel()]).T
    return points

# Define the sensor and register it with habitat
# For the sensor, we will register it with a custom name
@habitat.registry.register_sensor(name="position_sensor")
class AgentPositionSensor(habitat.Sensor):
    def __init__(self, sim, config, **kwargs: Any):
        super().__init__(config=config)

        self._sim = sim

    # Defines the name of the sensor in the sensor suite dictionary
    def _get_uuid(self, *args: Any, **kwargs: Any):
        return "agent_position"

    # Defines the type of the sensor
    def _get_sensor_type(self, *args: Any, **kwargs: Any):
        return habitat.SensorTypes.POSITION

    # Defines the size and range of the observations of the sensor
    def _get_observation_space(self, *args: Any, **kwargs: Any):
        return spaces.Box(
            low=np.finfo(np.float32).min,
            high=np.finfo(np.float32).max,
            shape=(3,),
            dtype=np.float32,
        )

    # This is called whenver reset is called or an action is taken
    def get_observation(
        self, observations, *args: Any, episode, **kwargs: Any
    ):
        sensor_states = self._sim.get_agent_state().sensor_states
        return (sensor_states['rgb'].position, sensor_states['rgb'].rotation)


class KeyboardAgent(habitat.Agent):
    def __init__(self, 
                 save_observations=True,
                 rgb_topic='/habitat/rgb/image',
                 depth_topic='/habitat/depth/image',
                 camera_info_topic='/habitat/rgb/camera_info',
                 path_topic='/true_path',
                 pose_topic='/true_pose',
                 odometry_topic='/true_odom',
                 publish_odom=False):
        #rospy.init_node('agent')
        self.save_observations = save_observations
        self.image_publisher = rospy.Publisher(rgb_topic, Image, latch=True, queue_size=100)
        self.depth_publisher = rospy.Publisher(depth_topic, Image, latch=True, queue_size=100)
        self.camera_info_publisher = rospy.Publisher(camera_info_topic, CameraInfo, latch=True, queue_size=100)
        self.true_path_publisher = rospy.Publisher(path_topic, Path, latch=True, queue_size=100)
        self.pose_publisher = rospy.Publisher(pose_topic, PoseStamped, latch=True, queue_size=100)
        self.publish_odom = publish_odom
        if self.publish_odom:
            self.odom_publisher = rospy.Publisher(odometry_topic, Odometry, latch=True, queue_size=100)
        self.image = Image()
        self.image.height = 240
        self.image.width = 320
        self.image.encoding = 'rgb8'
        self.image.is_bigendian = False
        self.depth = Image()
        self.depth.height = 240
        self.depth.width = 320
        self.depth.is_bigendian = True
        self.depth.encoding = 'mono8'
        self.camera_info = CameraInfo(width=320, height=240, D=D, K=K, R=R, P=P)
        self.cvbridge = CvBridge()
        self.true_trajectory = []
        self.predicted_trajectory = []
        self.map_path_subscriber = rospy.Subscriber('mapPath', Path, self.mappath_callback)
        self.slam_start_time = -1000
        self.slam_update_time = -1000
        self.is_started = False
        self.points = []
        self.true_positions = []
        self.true_rotations = []
        self.predicted_positions = []
        self.predicted_rotations = []
        self.rgbs = []
        self.depths = []
        self.actions = []
        self.true_timestamps = []
        self.predicted_timestamps = []
        self.speed = 0.
        self.twist = 1.
        self.time_of_publish = 0


    def mappath_callback(self, data):
        mappath_pose = data.poses[-1].pose
        x, y, z = mappath_pose.position.x, mappath_pose.position.y, mappath_pose.position.z
        xx, yy, zz, w = mappath_pose.orientation.x, mappath_pose.orientation.y, mappath_pose.orientation.z, mappath_pose.orientation.w
        cur_time = rospy.Time.now().secs + rospy.Time.now().nsecs * 1e-9
        eps = 1e-5
        if cur_time - self.slam_update_time > 30:
            self.slam_start_time = cur_time
            start_orientation = self.true_trajectory[-1].pose.orientation
            start_position = self.true_trajectory[-1].pose.position
            x_angle, z_angle, y_angle = tf.euler_from_quaternion([start_orientation.x, start_orientation.y, start_orientation.z, start_orientation.w])
            self.slam_start_angle = z_angle
            self.slam_start_x = start_position.x
            self.slam_start_y = start_position.y
            self.slam_start_z = start_position.z
            self.true_trajectory = []
        self.slam_update_time = cur_time
        self.predicted_trajectory.append(mappath_pose)
        self.predicted_positions.append([x, y, z])
        self.predicted_rotations.append([mappath_pose.orientation.w,
                                         mappath_pose.orientation.x,
                                         mappath_pose.orientation.y,
                                         mappath_pose.orientation.z])
        self.predicted_timestamps.append(cur_time)

    def reset(self):
        pass

    def get_actions_from_keyboard(self):
        keyboard_commands = [HabitatSimActions.MOVE_FORWARD] * int(self.speed)
        if keyboard.is_pressed('left'):
            keyboard_commands += [HabitatSimActions.TURN_LEFT] * max(int(self.twist), 1)
        if keyboard.is_pressed('right'):
            keyboard_commands += [HabitatSimActions.TURN_RIGHT] * max(int(self.twist), 1)
        if keyboard.is_pressed('up'):
            self.speed += 0.1
        if keyboard.is_pressed('down'):
            self.speed = max(self.speed - 0.2, 0)
        if keyboard.is_pressed('s'):
            self.speed = 0
        if keyboard.is_pressed('e'):
            self.twist += 0.2
        if keyboard.is_pressed('d'):
            self.twist = max(self.twist - 0.2, 0)
        return keyboard_commands

    def publish_rgb(self, image, start_time):
        #start_time = rospy.Time.now()
        self.image = self.cvbridge.cv2_to_imgmsg(image)
        self.image.encoding = 'rgb8'
        self.image.header.stamp = start_time
        self.image.header.frame_id = 'camera_link'
        self.image_publisher.publish(self.image)

    def publish_depth(self, depth, start_time):
        #start_time = rospy.Time.now()
        #print('Depth min and max:', depth.min(), depth.max())
        self.depth = self.cvbridge.cv2_to_imgmsg(depth * MAX_DEPTH)
        self.depth.header.stamp = start_time
        self.depth.header.frame_id = 'base_scan'
        self.depth_publisher.publish(self.depth)

    def publish_camera_info(self, start_time):
        #start_time = rospy.Time.now()
        self.camera_info.header.stamp = start_time
        self.camera_info_publisher.publish(self.camera_info)

    def publish_pose(self, pose, start_time):
        position, rotation = pose
        y, z, x = position
        cur_orientation = rotation
        cur_euler_angles = tf.euler_from_quaternion([cur_orientation.w, cur_orientation.x, cur_orientation.z, cur_orientation.y])
        cur_x_angle, cur_y_angle, cur_z_angle = cur_euler_angles
        cur_z_angle += np.pi
        cur_pose = PoseStamped()
        cur_pose.header.stamp = start_time
        cur_pose.pose.position.x = x
        cur_pose.pose.position.y = y
        cur_pose.pose.position.z = z
        cur_pose.pose.orientation = cur_orientation
        cur_pose.header.stamp = start_time
        self.pose_publisher.publish(cur_pose)

    def publish_true_path(self, pose, publish_odom):
        # count current coordinates and direction in global coords
        start_time = rospy.Time.now()
        position, rotation = pose
        y, z, x = position
        cur_orientation = rotation
        cur_euler_angles = tf.euler_from_quaternion([cur_orientation.w, cur_orientation.x, cur_orientation.z, cur_orientation.y])
        cur_x_angle, cur_y_angle, cur_z_angle = cur_euler_angles
        cur_z_angle += np.pi
        #print('Source position:', y, z, x)
        #print('Source quat:', cur_orientation.x, cur_orientation.y, cur_orientation.z, cur_orientation.w)
        #print('Euler angles:', cur_x_angle, cur_y_angle, cur_z_angle)
        #print('After tf:', tf.quaternion_from_euler(cur_x_angle, cur_y_angle, cur_z_angle))
        self.slam_update_time = start_time.secs + 1e-9 * start_time.nsecs
        if not self.is_started:
            self.is_started = True
            self.slam_start_angle = cur_z_angle
            #print("SLAM START ANGLE:", self.slam_start_angle)
            self.slam_start_x = x
            self.slam_start_y = y
            self.slam_start_z = z
        # if SLAM is running, transform global coords to RViz coords
        if self.publish_odom or (start_time.secs + start_time.nsecs * 1e-9) - self.slam_update_time < 30:
            rviz_x, rviz_y = inverse_transform(x, y, self.slam_start_x, self.slam_start_y, self.slam_start_angle)
            rviz_z = z - self.slam_start_z
            cur_quaternion = tf.quaternion_from_euler(0, 0, cur_z_angle - self.slam_start_angle)
            #print('Rotated quat:', cur_quaternion)
            cur_orientation.w = cur_quaternion[0]
            cur_orientation.x = cur_quaternion[1]
            cur_orientation.y = cur_quaternion[2]
            cur_orientation.z = cur_quaternion[3]
            x, y, z = rviz_x, rviz_y, rviz_z
        self.true_positions.append(np.array([x, y, z]))
        self.true_rotations.append(cur_quaternion)
        # add current point to path
        cur_pose = PoseStamped()
        cur_pose.header.stamp = start_time
        cur_pose.pose.position.x = x
        cur_pose.pose.position.y = y
        cur_pose.pose.position.z = z
        cur_pose.pose.orientation = cur_orientation
        self.true_trajectory.append(cur_pose)
        # publish the path
        true_path = Path()
        #print(start_time)
        true_path.header.stamp = start_time
        true_path.header.frame_id = 'map'
        true_path.poses = self.true_trajectory
        self.true_path_publisher.publish(true_path)
        # publish odometry
        if self.publish_odom:
            odom = Odometry()
            odom.header.stamp = start_time
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose = cur_pose.pose
            self.odom_publisher.publish(odom)

    def act(self, observations, env):
        # publish all observations to ROS
        start_time = rospy.Time.now()
        start_time_seconds = start_time.secs + start_time.nsecs * 1e-9
        #pcd = get_local_pointcloud(observations['rgb'], observations['depth'])
        #print(pcd.shape)
        if self.save_observations:
            self.points.append(pcd)
            #print(observations['rgb'].shape, observations['depth'].shape)
            self.rgbs.append(observations['rgb'])#.reshape((240 * 320, 3)))
            self.depths.append(observations['depth'])
            cur_time = rospy.Time.now()
            self.true_timestamps.append(cur_time.secs + 1e-9 * cur_time.nsecs)
        #if start_time_seconds - self.time_of_publish > 1. / rate:
        self.publish_rgb(observations['rgb'], start_time)
        self.publish_depth(observations['depth'], start_time)
        self.publish_camera_info(start_time)
        self.publish_true_path(observations['agent_position'], self.publish_odom)
        self.publish_pose(observations['agent_position'], start_time)
        self.time_of_publish = start_time_seconds
        # receive command from keyboard and move
        actions = self.get_actions_from_keyboard()
        if len(actions) > 0:
            for action in actions[:-1]:
                env.step(action)
        cur_time = rospy.Time.now()
        cur_time_seconds = cur_time.secs + cur_time.nsecs * 1e-9
        print(cur_time_seconds)
        # make act time (1/rate) seconds
        time_left = cur_time_seconds - start_time_seconds
        print('Action took {} seconds'.format(time_left))
        print('Speed coef is', self.speed)
        print('Twist coef is', self.twist)
        #rospy.sleep(1. / rate - time_left)
        #self.actions.append(str(action))
        if len(actions) > 0:
            return actions[-1]
        else:
            return HabitatSimActions.STOP


def build_pointcloud(sim, discretization=0.05, grid_size=500, num_samples=20000):
    range_x = (np.inf, -np.inf)
    range_y = (np.inf, -np.inf)
    range_z = (np.inf, -np.inf)
    pointcloud = set()
    for i in range(num_samples):
        point = sim.sample_navigable_point()
        x, z, y = point
        z = np.random.random() * 3
        range_x = (min(range_x[0], x), max(range_x[1], x))
        range_y = (min(range_y[0], y), max(range_y[1], y))
        range_z = (min(range_z[0], z), max(range_z[1], z))
    for x in tqdm(np.linspace(range_x[0], range_x[1], grid_size)):
        for y in np.linspace(range_y[0], range_y[1], grid_size):
            for z in np.linspace(range_z[0], range_z[1], 100):
                closest_obstacle_point = sim._sim.pathfinder.closest_obstacle_surface_point(np.array([x, z, y])).hit_pos
                x_, z_, y_ = closest_obstacle_point
                x_ = np.round(x_ / discretization) * discretization
                y_ = np.round(y_ / discretization) * discretization
                z_ = np.round(z_ / discretization) * discretization
                pointcloud.add((x_, y_, z_))
    return np.array(list(pointcloud))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--task-config", type=str, default="configs/tasks/pointnav_mp3d.yaml")
    parser.add_argument("--publish-odom", type=bool, default=True)
    parser.add_argument("--create-map", type=bool, default=False)
    parser.add_argument("--save-observations", type=bool, default=False)
    parser.add_argument("--preset-trajectory", type=bool, default=False)
    args = parser.parse_args()
    # Now define the config for the sensor
    config = habitat.get_config(args.task_config)
    config.defrost()
    config.TASK.AGENT_POSITION_SENSOR = habitat.Config()
    # Use the custom name
    config.TASK.AGENT_POSITION_SENSOR.TYPE = "position_sensor"
    config.TASK.AGENT_POSITION_SENSOR.ANSWER_TO_LIFE = 42
    # Add the sensor to the list of sensors in use
    config.TASK.SENSORS.append("AGENT_POSITION_SENSOR")
    config.freeze()

    agent = KeyboardAgent(args.save_observations)
    env = habitat.Env(config=config)
    print('A point of the env:', env.sim.sample_navigable_point())
    if args.create_map:
        top_down_map = maps.get_topdown_map(env.sim, map_resolution=(5000, 5000))
        recolor_map = np.array([[0, 0, 0], [128, 128, 128], [255, 255, 255]], dtype=np.uint8)
        range_x = np.where(np.any(top_down_map, axis=1))[0]
        range_y = np.where(np.any(top_down_map, axis=0))[0]
        padding = int(np.ceil(top_down_map.shape[0] / 125))
        range_x = (
            max(range_x[0] - padding, 0),
            min(range_x[-1] + padding + 1, top_down_map.shape[0]),
        )
        range_y = (
            max(range_y[0] - padding, 0),
            min(range_y[-1] + padding + 1, top_down_map.shape[1]),
        )
        top_down_map = top_down_map[
            range_x[0] : range_x[1], range_y[0] : range_y[1]
        ]
        top_down_map = recolor_map[top_down_map]
        imsave('top_down_map.png', top_down_map)
    observations = env.reset()
    if not args.preset_trajectory:
        while not keyboard.is_pressed('q'):
            action = agent.act(observations, env)
            observations = env.step(action)
            rate.sleep()
    else:
        fin = open('actions.txt', 'r')
        actions = [int(x) for x in fin.readlines()]
        for action in actions:
            agent.act(observations)
            observations = env.step(action)
        fin.close()
    if args.save_observations:
        print(np.array(agent.rgbs).shape)
        with h5py.File('observations.hdf5', 'w') as f:
            f.create_dataset("points", data=np.array(agent.points))
            f.create_dataset("true_positions", data=np.array(agent.true_positions))
            f.create_dataset("true_rotations", data=np.array(agent.true_rotations))
            f.create_dataset("predicted_positions", data=np.array(agent.predicted_positions))
            f.create_dataset("predicted_rotations", data=np.array(agent.predicted_rotations))
            f.create_dataset("rgb", data=np.array(agent.rgbs))
            f.create_dataset("depth", data=np.array(agent.depths))
            f.create_dataset("true_timestamps", data=np.array(agent.true_timestamps))
            f.create_dataset("predicted_timestamps", data=np.array(agent.predicted_timestamps))
        print(np.array(agent.points).shape, np.array(agent.rgbs).shape)
        fout = open('actions.txt', 'w')
        for action in agent.actions:
            print(action, file=fout)
        fout.close()


if __name__ == "__main__":
    main()
