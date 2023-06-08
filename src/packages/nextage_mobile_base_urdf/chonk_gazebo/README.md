# chonk_gazebo

This package contains the scripts, configuration and launch files needed to simulate the chonk robot.

<p align="center">
  <img src="https://github.com/ipab-slmc/chonk_gazebo/blob/main/images/chonk_gazebo.gif" height="400">
</p>

## Simulation
Enter the following in a terminal to launch gazebo:
```
$   roslaunch chonk_gazebo chonk.launch
```
To move the base, in a separate terminal load rqt and select the Topics -> Easy Message Publisher plugin.
```
$   rqt
```
From the drop down list select the ```/chonk/donkey_velocity_controller/cmd_vel``` topic, and publish to the linear x, linear y & angular z topics by dragging the slider.
<p align="center">
  <img src="https://github.com/ipab-slmc/donkey_gazebo/blob/main/images/donkey_control_rqt.png">
</p>

To control the Nextage joints, in rqt select the Controller Manager and Joint Trajectory Controller plugins. In the manager unload all Nextage controllers and load the trajectory controller, and then you can control multiple joints at once.
<p align="center">
  <img src="https://github.com/ipab-slmc/chonk_gazebo/blob/main/images/joint_trajectory_controller.png">
</p>
