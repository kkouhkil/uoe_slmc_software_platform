# donkey_description

This package contains the ```.xacro``` and mesh files that generate the ```.urdf``` file needed to describe and simulate the donkey mobile base robot.

<p align="center">
  <img src="https://github.com/ipab-slmc/donkey_description/blob/main/images/donkey_gazebo.png" height="300">
</p>

## Package overview
- ```base.xacro``` creates the macros needed to generate a generic mobile base with meccanum wheels (description below).
- ```donkey.xacro``` creates a **donkey_base macro** which instantiates the **mecanum_base macro** and four **mecanum_wheel macros** with the specific donkey mobile base measurements. The argument ```donkey_gazebo_ros_control``` should be true if you are simulating the base alone.
- ```donkey.urdf.xacro``` instantiates a **donkey_base macro** and a **mecanum_base_mount macro** with a dummy link since no other robot is attached.
- In the ```mesh``` folder, the original STL model (22.8 MB) is included. Using Blendr, many of the inside faces and vertices were deleted to reduce the file size, and colour was added by adding textures to the faces. The file size was then reduced in size by half again by decimating half the faces with an in-built tool.

## Generic macros (base.xacro)

The **mecanum_wheel macro** creates the wheel link with cylindrical visual and intertial geometry, and a continous joint it can rotate around. It also instantiates a **gazebo_mecanum_link macro**, loads the ```ridgeback_simulator``` mecanum wheel controller for gazebo, and defines the transmission element.

The **gazebo_mecanum_link macro**  creates the corresponding passive wheel link and joint needed to simulate the mecanum motion. In simulation, the 4x passive wheels possess spherical collision geometry with a singular contact point to the ground, enabling mecanum motion.

The **mecanum_base macro** creates the ```donkey_base``` link. It has box inertial and collision geometry, and the visual geometry is loaded from a mesh file.

The **mecanum_base_mount macro** creates the joint connecting the ```donkey_base``` link to the ```base_link```. It expects a ```child``` argument, so if there is an attached robot this is where you pass the connecting link. If there is not, you can pass a dummy link.

## Visualisation
A quick way to visualise the robot description is with a TF tree. First launch the robot model in gazebo with:

```
$   roslaunch donkey_gazebo donkey.launch
```
You can then view the TF tree in rqt, or by adding ```alias tf2='cd /var/tmp && rosrun tf2_tools view_frames.py && evince frames.pdf &'``` to your ```~/.bashrc```, sourcing, then entering:

```
$   tf2
```
The result should look like this:
<p align="center">
  <img src="https://github.com/ipab-slmc/donkey_description/blob/main/images/donkey_tf_tree.png" height="550">
</p>
