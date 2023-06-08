#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

bool time_now;

void odomCallback(const nav_msgs::OdometryConstPtr& msg_in)
{
  static tf::TransformBroadcaster br;
 
  ros::Time t;  
  if(time_now)
  {
    t = ros::Time::now();
  }                
  else
  {
    t = msg_in->header.stamp;
  }
  
  br.sendTransform( tf::StampedTransform(
                    tf::Transform(
                    tf::Quaternion(msg_in->pose.pose.orientation.x,
                                   msg_in->pose.pose.orientation.y,
                                   msg_in->pose.pose.orientation.z,
                                   msg_in->pose.pose.orientation.w), 
                    tf::Vector3(msg_in->pose.pose.position.x,
                                   msg_in->pose.pose.position.y,
                                   msg_in->pose.pose.position.z)),
                    t, "odom","base_link"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odo_to_tf_node");
  ros::NodeHandle nh("~");  

  std::string odo_topic;
  int odo_queue_size;
  
  // Read parameters
  nh.param<std::string>("odo_topic", odo_topic, "/odom");
  nh.param<bool>("time_now", time_now, false);  
  nh.param<int>("odo_queue_size", odo_queue_size, 20);
   
  // Create subscriber for odometry message    
  ros::Subscriber odom_sub = nh.subscribe(odo_topic, odo_queue_size, &odomCallback);

  ros::spin();
  return 0;
};
