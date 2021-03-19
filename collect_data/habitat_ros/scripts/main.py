#! /usr/bin/env python

import rospy
import habitat
from keyboard_agent import KeyboardAgent
from shortest_path_follower_agent import ShortestPathFollowerAgent
from custom_sensors import AgentPositionSensor
from publishers import HabitatObservationPublisher

DEFAULT_RATE = 30
DEFAULT_AGENT_TYPE = 'keyboard'


def main():
    # Initialize ROS node and take arguments
    rospy.init_node('habitat_ros_node')
    task_config = rospy.get_param('~task_config')
    rate_value = rospy.get_param('~rate', DEFAULT_RATE)
    agent_type = rospy.get_param('~agent_type', DEFAULT_AGENT_TYPE)
    rgb_topic = rospy.get_param('~rgb_topic', None)
    depth_topic = rospy.get_param('~depth_topic', None)
    camera_info_topic = rospy.get_param('~camera_info_topic', None)
    true_pose_topic = rospy.get_param('~true_pose_topic', None)
    camera_info_file = rospy.get_param('~camera_calib', None)
    rate = rospy.Rate(rate_value)
    publisher = HabitatObservationPublisher(rgb_topic, 
                                            depth_topic, 
                                            camera_info_topic, 
                                            true_pose_topic,
                                            camera_info_file)

    # Now define the config for the sensor
    config = habitat.get_config(task_config)
    config.defrost()
    config.TASK.AGENT_POSITION_SENSOR = habitat.Config()
    config.TASK.AGENT_POSITION_SENSOR.TYPE = "position_sensor"
    config.TASK.AGENT_POSITION_SENSOR.ANSWER_TO_LIFE = 42
    config.TASK.SENSORS.append("AGENT_POSITION_SENSOR")
    config.freeze()

    # Initialize the agent and environment
    env = habitat.Env(config=config)
    if agent_type == 'keyboard':
       agent = KeyboardAgent()
    elif agent_type == 'shortest_path_follower':
        goal_radius = 0.25
        agent = ShortestPathFollowerAgent(env, goal_radius)
    else:
        print('AGENT TYPE {} IS NOT DEFINED!!!'.format(agent_type))
        return

    # Run the simulator with agent
    observations = env.reset()
    while not rospy.is_shutdown():
        publisher.publish(observations)
        action = agent.act(observations, env)
        observations = env.step(action)
        rate.sleep()


if __name__ == '__main__':
    main()