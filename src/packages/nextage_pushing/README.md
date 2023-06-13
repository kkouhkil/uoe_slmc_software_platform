# Dependencies

Use [`wstool`](http://wiki.ros.org/wstool) to handle dependencies.
If you already have a rosinstall file in the src/ directory, from the workspace directory, merge the rosinstall files:
```
wstool merge -t src src/iiwa_wiping/dependencies.rosinstall
```
Then update the workspace:
```
wstool update -t src
```

# Install
Clone into workspace and resolve dependencies:
```sh
rosdep install --from-paths src --ignore-src -y
catkin build
source devel/setup.bash
```

# Usage
#### Launch files depend on each other. As such, launch files are recommended to be launched in the following order:

1. First launch pybullet, gazebo, or robot, depending on whether you would like a pybullet or gazebo 3D simulation to view the robot, its movements and its interactions with objects. Otherwise, if you intend deploy on robotic hardware, the robot.launch file can be used:
```sh
roslaunch nextage_pushing pybullet.launch
```
```sh
roslaunch nextage_pushing gazebo.launch
```
```sh
roslaunch nextage_pushing robot.launch
```

2. Action Servers - launches several action services for example sending the robot to specified configuration or pose, or tele-operating the end-effector. Waits for client to be launched (based on actionlib http://wiki.ros.org/actionlib):
```sh
roslaunch nextage_pushing action_servers.launch
```

3. For teleoperation of the robot in 2D or 3D, the following will launch the action-client:
```sh
roslaunch nextage_pushing teleop2d.launch
```
```sh
roslaunch nextage_pushing teleop3d.launch
```

 RViz Visualizer (has no interdependency with any other launch files) - it launches the rviz visualizer tool to view the robot with more specific information about its motions (displays axes of rotation/translation for actuators, force feedback, etc.):
```sh
roslaunch nextage_pushing rviz.launch
```
