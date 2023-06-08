## chonk_description

This package creates a robot description using the existing [nextagea_description](https://github.com/ipab-slmc/nextagea/tree/master/nextagea_description) and [donkey_description](https://github.com/ipab-slmc/donkey_description) code.

<p align="center">
  <img src="https://github.com/ipab-slmc/chonk_description/blob/main/images/chonk_gazebo.png">
</p>

## Package overview

In the ```chonk.urdf.xacro``` file, a **nextagea robot**, **donkey base**, and **base mount macro** are instantiated.
- The only difference to [donkey_description](https://github.com/ipab-slmc/donkey_description) is that now the ```child``` argument to the **base mount macro** is set to be the ```WAIST``` link of nextagea.
- ```nextagea_gazebo_ros_control``` and ```donkey_gazebo_ros_control``` arguments are both set to false, and instead a new ```gazebo_ros_control``` plugin is loaded under the ```chonk``` namespace.
- Cameras, grippers, sensors etc. can be added to the robot as described in [nextage_extensions](https://github.com/ipab-slmc/nextage_extensions).

## Visualisation
A quick way to visualise the robot description is with a TF tree. First launch the robot model in gazebo with:

```
$   roslaunch chonk_gazebo chonk.launch
```
You can then view the TF tree in rqt, or by adding ```alias tf2='cd /var/tmp && rosrun tf2_tools view_frames.py && evince frames.pdf &'``` to your ```~/.bashrc```, sourcing, then entering:

```
$   tf2
```
The result should look like this:
<p align="center">
  <img src="https://github.com/ipab-slmc/chonk_description/blob/main/images/chonk_tf_tree.png">
</p>
