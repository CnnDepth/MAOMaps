# habitat_ros
Integrate Habitat simulator with ROS

## Requirements

* ROS version at least Kinetic
* CMake version at least 2.8.3
* Python version at least 3.4
* Habitat simulator
* Python packages: numpy, gym, keyboard, cv_bridge

## Installation

1) Install ROS on your system ([https://www.ros.org/install/](https://www.ros.org/install/)).

2) Clone this repo into your ROS workspace (e.g. `~/catkin_ws`), and run `catkin_make`.

3) Download and build [Habitat-sim](https://github.com/facebookresearch/habitat-sim) and [Habitat-lab](https://github.com/facebookresearch/habitat-lab).

4) Install required Python packages: `pip install -r requirements.txt`

5) In file `habitat-lab/habitat/tasks/nav/nav.py`, find method `step` in class `StopAction`, and change line `task.is_stop_called = True` to `task.is_stop_called=False` in code of this method.

6) Rebuild habitat-lab.

## Launch

1) Open terminal and enable root (e.g. via `sudo su` command).

2) Run agent in simulator with keyboard operation via `roslaunch habitat_ros keyboard_agent.launch`.

## Node parameters

* rgb_topic (default: None) - topic to publish RGB image from simulator
* depth_topic (default: None) - topic to publish depth image from simulator
* camera_info_topic (default: None) - topic to publish information about camera calibration (read from file)
* camera_calib (default: None) - path to camera calibration file
