# donkey_gazebo

This package contains the scripts, configuration and launch files needed to simulate the donkey mobile base robot.

<p align="center">
  <img src="https://github.com/ipab-slmc/donkey_gazebo/blob/main/images/donkey_control.gif" height="400">
</p>

## Simulation
Enter the following in a terminal to launch gazebo:
```
$   roslaunch donkey_gazebo donkey.launch
```
To move the base, in a separate terminal load rqt and select the Topics -> Easy Message Publisher plugin.
```
$   rqt
```
From the drop down list select the ```/donkey/donkey_velocity_controller/cmd_vel``` topic, and publish to the linear x, linear y & angular z topics by dragging the slider.
<p align="center">
  <img src="https://github.com/ipab-slmc/donkey_gazebo/blob/main/images/donkey_control_rqt.png">
</p>

## Localisation
In addition to simulating the mobile base velocity controllers, this package also includes files for state estimation and localisation. ```donkey.launch``` launches ```state_estimation.launch```, which starts an EKF localisation node and a state estimation node. Currently the only input to the state estimator is the wheel odometry recorded by the donkey velocity controller.
