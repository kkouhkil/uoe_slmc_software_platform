TF server
=========

Package for publishing robot specific `tf2` or `tf`  messages. This package contains `odo_to_tf2_node` and `odo_to_tf_node` that subscribe to dynamic odometry messages and publishes 
them as `tf2` or `tf` messages respectively. Static transformations are broadcasted using `static_transform_publisher` node from `tf2_ros` of `tf` package.

Usage
-----
> roslaunch tf_server tf_server_gimbot_tf.launch odo_topic:=<put your topic here> 

For example:
> roslaunch tf_server tf_server_gimbot_tf.launch odo_topic:=/gim_localization_node/filtered

Parameters
----------
- **odo_topic**: topic for the subscribed nav_msgsg/Odometry message that is converted to tf message. Default: /ndt_mcl.
