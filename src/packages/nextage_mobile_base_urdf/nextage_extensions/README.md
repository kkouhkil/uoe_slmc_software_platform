## Nextage Extensions

This package contains additional sensor and actuator files to be used for NextageA gazebo simulation. See the [Nextage Wiki](https://github.com/ipab-slmc/nextagea_wiki/wiki/Nextage) for more detail about the extensions and Gazebo pictures.

The basic NextageA robot is defined in ```nextagea/nextagea_description/urdf/nextagea.urdf.xacro``` by instantiating a robot, base, and hand tools as:
```
<xacro:nextagea_robot/>
<xacro:nextagea_base/>
<xacro:nextagea_tools prefix="L" use_cables="true"/>
<xacro:nextagea_tools prefix="R" use_cables="false"/>
```

To add sensors and actuators to your project, simply include the desired xacro file from [Nextage Extensions](https://github.com/ipab-slmc/nextage_extensions/tree/main/urdf) in the main NextageA description file ```nextagea.urdf.xacro```, along with the macro. The necessary input code and corresponding simulator output are shown below.

# Input parameters
For more information on xacro, see [here](https://github.com/ros/xacro/wiki) & [here](http://wiki.ros.org/xacro).

When instantiating the macros below, you will need to specify some arguments. The ```prefix``` argument is the given name, ```parent``` is the link it connects to, and ```origin``` is a block parameter which specifies where the element goes in relation to the link's origin.

# Dependencies

Make sure you have all the local repository dependencies through:
```
vcs import src < src/nextage_extensions/dependencies.rosinstall
```
In ubuntu you can get the vcs tool through:
```
sudo apt install python3-vsctool
```
Finally, use rosdep to get system dependencies:
```
rosdep install --from-paths src --ignore-src -r -y
```

# Sensors

**Intel Realsense**
```
<xacro:include filename="$(find nextage_extensions)/urdf/intel_realsense.xacro" />

<xacro:nx_intel_realsense_mount prefix="intel_realsense_"/>
```
or
```
<xacro:include filename="$(find nextage_extensions)/urdf/intel_realsense_azure_topics.xacro" />

<xacro:nx_intel_realsense_azure_topics_mount prefix="intel_realsense_azure_topics_"/>
```

**Microsoft Kinect Camera & Mount**
```
<xacro:include filename="$(find nextage_extensions)/urdf/azure_kinect.xacro" />

<xacro:nx_kinect_mount prefix="generic_"/>
```

**Microsoft Kinect Camera Only**
The camera has its own dedicated macro. Note that to use this without the mount macro, you will need to configure the necessary links etc.
```
<xacro:include filename="$(find nextage_extensions)/urdf/azure_kinect.xacro" />

<xacro:azure_kinect prefix="generic_"/>
```

**Generic IMU**
```
<xacro:include filename="$(find nextage_extensions)/urdf/imu.xacro" />

<xacro:imu prefix="generic_" parent="base_link">
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
</xacro:imu>
```

**HEX-E Sensor**
```
<xacro:include filename="$(find nextage_extensions)/urdf/hexe.xacro" />

<xacro:hex_e_ft_sensor prefix="LARM_" topic_name="ft_left/raw/data" parent="LARM_JOINT5_Link">
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:hex_e_ft_sensor>

<xacro:hex_e_ft_sensor prefix="RARM_" topic_name="ft_right/raw/data" parent="LARM_JOINT5_Link">
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:hex_e_ft_sensor>
```

**ATI F/T Sensor**
```
<xacro:include filename="$(find nextage_extensions)/urdf/ati.xacro" />

<xacro:ati_ft_sensor parent="LARM_JOINT5_Link" prefix="LARM_" topic_name="ft_left/raw/data">
    <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:ati_ft_sensor>

<xacro:ati_ft_sensor parent="RARM_JOINT5_Link" prefix="RARM_" topic_name="ft_right/raw/data">
    <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:ati_ft_sensor>
```

# Actuators

**Robotiq 85 Gripper**
```
<xacro:include filename="$(find nextage_extensions)/urdf/robotiq_85_gripper.xacro" />

<xacro:robotiq_85_gripper prefix="right_" parent="RARM_JOINT5_Link">
    <origin xyz="0 0 0" rpy="0 3.1415 0"/>
</xacro:robotiq_85_gripper>
```

**Robotiq 140 Gripper**
```
<xacro:include filename="$(find nextage_extensions)/urdf/robotiq_140_gripper.xacro" />

<xacro:robotiq_140_gripper prefix="left_" parent="LARM_JOINT5_Link">
  <origin xyz="0 0 0" rpy="0 3.1415 0"/>
</xacro:robotiq_140_gripper>
```

**Robolimb**
```
<xacro:include filename="$(find nextage_extensions)/urdf/robolimb.xacro" />

<xacro:robolimb_attach prefix="right_" parent="RARM_JOINT5_Link">
  <origin xyz="0 0 -0.12" rpy="3.14 0 -1.57"/>
</xacro:robolimb_attach>
```

# Adapters

**OnRobot Quick Changer**
```
<xacro:include filename="$(find nextage_extensions)/urdf/tool_changer.xacro" />

<xacro:tool_changer prefix="LARM_" parent="LARM_ft_link">
    <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:tool_changer>

<xacro:tool_changer prefix="RARM_" parent="RARM_ft_link">
    <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:tool_changer>
```
