#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

bool time_now;

void odomCallback(const nav_msgs::OdometryConstPtr& msg_in)
{

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  ros::Time t;
  
  if(time_now)
  {
    t = ros::Time::now();
  }                
  else
  {
    t = msg_in->header.stamp;
  }
  
  transformStamped.header.stamp = t;
//  transformStamped.header.frame_id = msg_in->header.frame_id;
  transformStamped.header.frame_id = "world";
//  transformStamped.child_frame_id = msg_in->child_frame_id;
  transformStamped.child_frame_id = "base_link";

  transformStamped.transform.translation.x = msg_in->pose.pose.position.x;
  transformStamped.transform.translation.y = msg_in->pose.pose.position.y;
  transformStamped.transform.translation.z = msg_in->pose.pose.position.z;

  transformStamped.transform.rotation.x = msg_in->pose.pose.orientation.x;
  transformStamped.transform.rotation.y = msg_in->pose.pose.orientation.y;
  transformStamped.transform.rotation.z = msg_in->pose.pose.orientation.z;
  transformStamped.transform.rotation.w = msg_in->pose.pose.orientation.w;

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odo_to_tf2_node");
  ros::NodeHandle nh("~");

  std::string odo_topic;  
  int odo_queue_size;
  
  // Read parameters
  ros::param::get("~odo_topic", odo_topic);
  ros::param::get("~odo_queue_size", odo_queue_size);
  ros::param::get("~time_now", time_now);
       
  // Create subscriber for odometry message    
  ros::Subscriber odom_sub = nh.subscribe(odo_topic, odo_queue_size, &odomCallback);

  ros::spin();
  return 0;
};
