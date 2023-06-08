/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Irstea
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Irstea nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <cmath>

#include <tf/transform_datatypes.h>

#include <boost/assign.hpp>
#include <sensor_msgs/JointState.h>

#include <four_wheel_carlike_steering_controller/four_wheel_carlike_steering_controller.h>
#include <urdf_geometry_parser/urdf_geometry_parser.h>


namespace four_wheel_carlike_steering_controller{

  FourWheelCarlikeSteeringController::FourWheelCarlikeSteeringController()
    : command_struct_twist_()
    ,last_update_timestamp_(0.0)
    , command_struct_four_wheel_steering_()
    , track_(0.0)
    , wheel_steering_y_offset_(0.0)
    , wheel_radius_(0.0)
    , wheel_base_(0.0)
    , cmd_vel_timeout_(0.5)
    , base_frame_id_("base_link")
    , enable_odom_tf_(true)
    , enable_twist_cmd_(false)
    , wheel_joints_size_(0.0)
    , publish_wheel_joint_controller_state_(true)
    ,reset_signal(0.0)
  {
    rtObj.initialize();
  }

  void FourWheelCarlikeSteeringController::init_strart(const ros::Time& time)
  {
    // Reset accumulators and timestamp:
    last_update_timestamp_ = time;
  }

  bool FourWheelCarlikeSteeringController::init(hardware_interface::RobotHW *robot_hw,
                                         ros::NodeHandle& root_nh,
                                         ros::NodeHandle &controller_nh)
  {
    const std::string complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    name_ = complete_ns.substr(id + 1);

    // Get joint names from the parameter server
    std::vector<std::string> front_wheel_names, rear_wheel_names;
    if (!getWheelNames(controller_nh, "front_wheel", front_wheel_names) ||
        !getWheelNames(controller_nh, "rear_wheel", rear_wheel_names))
    {
      return false;
    }

    if (front_wheel_names.size() != rear_wheel_names.size())
    {
      ROS_ERROR_STREAM_NAMED(name_,
          "#front wheels (" << front_wheel_names.size() << ") != " <<
          "#rear wheels (" << rear_wheel_names.size() << ").");
      return false;
    }
    else if (front_wheel_names.size() != 2)
    {
      ROS_ERROR_STREAM_NAMED(name_,
          "#two wheels by axle (left and right) is needed; now : "<<front_wheel_names.size()<<" .");
      return false;
    }
    else
    {
      front_wheel_joints_.resize(front_wheel_names.size());
      rear_wheel_joints_.resize(front_wheel_names.size());
    }

    wheel_joints_size_ = front_wheel_joints_.size();

    // Get steering joint names from the parameter server
    std::vector<std::string> front_steering_names, rear_steering_names;
    if (!getWheelNames(controller_nh, "front_steering", front_steering_names) ||
        !getWheelNames(controller_nh, "rear_steering", rear_steering_names))
    {
      return false;
    }

    if (front_steering_names.size() != rear_steering_names.size())
    {
      ROS_ERROR_STREAM_NAMED(name_,
          "#left steerings (" << front_steering_names.size() << ") != " <<
          "#right steerings (" << rear_steering_names.size() << ").");
      return false;
    }
    else if (front_steering_names.size() != 2)
    {
      ROS_ERROR_STREAM_NAMED(name_,
          "#two steering by axle (left and right) is needed; now : "<<front_steering_names.size()<<" .");
      return false;
    }
    else
    {
      front_steering_joints_.resize(front_steering_names.size());
      rear_steering_joints_.resize(front_steering_names.size());
    }

    // Odometry related:
    double publish_rate=50;
    //controller_nh.param("publish_rate", publish_rate, 50.0);
    ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                          << publish_rate << "Hz.");
    publish_period_ = ros::Duration(1.0 / publish_rate);

    controller_nh.param("open_loop", open_loop_, open_loop_);

    int velocity_rolling_window_size = 10;
    controller_nh.param("velocity_rolling_window_size", velocity_rolling_window_size, velocity_rolling_window_size);
    //ROS_INFO_STREAM_NAMEcurr_cmd_twistD(name_, "Velocity rolling window size of " << velocity_rolling_window_size << ".");

    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

    // Twist command related:
    controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
    ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than " << cmd_vel_timeout_ << "s.");

    controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

    controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
    ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_?"enabled":"disabled"));

    // Velocity and acceleration limits:
    controller_nh.param("linear/x/has_velocity_limits"    , limiter_lin_.has_velocity_limits    , limiter_lin_.has_velocity_limits    );
    controller_nh.param("linear/x/has_acceleration_limits", limiter_lin_.has_acceleration_limits, limiter_lin_.has_acceleration_limits);
    controller_nh.param("linear/x/max_velocity"           , limiter_lin_.max_velocity           ,  limiter_lin_.max_velocity          );
    controller_nh.param("linear/x/min_velocity"           , limiter_lin_.min_velocity           , -limiter_lin_.max_velocity          );
    controller_nh.param("linear/x/max_acceleration"       , limiter_lin_.max_acceleration       ,  limiter_lin_.max_acceleration      );
    controller_nh.param("linear/x/min_acceleration"       , limiter_lin_.min_acceleration       , -limiter_lin_.max_acceleration      );

    controller_nh.param("angular/z/has_velocity_limits"    , limiter_ang_.has_velocity_limits    , limiter_ang_.has_velocity_limits    );
    controller_nh.param("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits, limiter_ang_.has_acceleration_limits);
    controller_nh.param("angular/z/max_velocity"           , limiter_ang_.max_velocity           ,  limiter_ang_.max_velocity          );
    controller_nh.param("angular/z/min_velocity"           , limiter_ang_.min_velocity           , -limiter_ang_.max_velocity          );
    controller_nh.param("angular/z/max_acceleration"       , limiter_ang_.max_acceleration       ,  limiter_ang_.max_acceleration      );
    controller_nh.param("angular/z/min_acceleration"       , limiter_ang_.min_acceleration       , -limiter_ang_.max_acceleration      );

    // Publish wheel data:
    controller_nh.param("publish_wheel_joint_controller_state", publish_wheel_joint_controller_state_, publish_wheel_joint_controller_state_);

    // If either parameter is not available, we need to look up the value in the URDF
    bool lookup_track = !controller_nh.getParam("track", track_);
    bool lookup_wheel_radius = !controller_nh.getParam("wheel_radius", wheel_radius_);
    bool lookup_wheel_base = !controller_nh.getParam("wheel_base", wheel_base_);

    urdf_geometry_parser::UrdfGeometryParser uvk(root_nh, base_frame_id_);
    if(lookup_track)
      if(!uvk.getDistanceBetweenJoints(front_wheel_names[0], front_wheel_names[1], track_))
        return false;
      else
        controller_nh.setParam("track",track_);

    if(!uvk.getDistanceBetweenJoints(front_steering_names[0], front_wheel_names[0], wheel_steering_y_offset_))
      return false;
    else
      controller_nh.setParam("wheel_steering_y_offset",wheel_steering_y_offset_);

    if(lookup_wheel_radius)
      if(!uvk.getJointRadius(front_wheel_names[0], wheel_radius_))
        return false;
      else
        controller_nh.setParam("wheel_radius",wheel_radius_);

    if(lookup_wheel_base)
      if(!uvk.getDistanceBetweenJoints(front_wheel_names[0], rear_wheel_names[0], wheel_base_))
        return false;
      else
        controller_nh.setParam("wheel_base",wheel_base_);

    // Regardless of how we got the separation and radius, use them
    // to set the odometry parameters
    odometry_.setWheelParams(track_-2*wheel_steering_y_offset_, wheel_steering_y_offset_, wheel_radius_, wheel_base_);
    //ROS_INFO_STREAM_Ncurr_cmd_twistAMED(name_, "Odometry params : track " << track_ << ", wheel radius " << wheel_radius_ << ", wheel base " << wheel_base_ << ", wheel steering offset " << wheel_steering_y_offset_);

    setOdomPubFields(root_nh, controller_nh);

    // Wheel joint controller state:
    if (publish_wheel_joint_controller_state_)
    {
      controller_state_pub_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState>(controller_nh, "wheel_joint_controller_state", 100));

      const size_t num_wheels = wheel_joints_size_ * 4;

      controller_state_pub_->msg_.joint_names.resize(num_wheels);

      controller_state_pub_->msg_.desired.positions.resize(num_wheels);
      controller_state_pub_->msg_.desired.velocities.resize(num_wheels);
      controller_state_pub_->msg_.desired.accelerations.resize(num_wheels);
      controller_state_pub_->msg_.desired.effort.resize(num_wheels);

      controller_state_pub_->msg_.actual.positions.resize(num_wheels);
      controller_state_pub_->msg_.actual.velocities.resize(num_wheels);
      controller_state_pub_->msg_.actual.accelerations.resize(num_wheels);
      controller_state_pub_->msg_.actual.effort.resize(num_wheels);

      controller_state_pub_->msg_.error.positions.resize(num_wheels);
      controller_state_pub_->msg_.error.velocities.resize(num_wheels);
      controller_state_pub_->msg_.error.accelerations.resize(num_wheels);
      controller_state_pub_->msg_.error.effort.resize(num_wheels);

      for (size_t i = 0; i < wheel_joints_size_; ++i)
      {
        controller_state_pub_->msg_.joint_names[i] = front_wheel_names[i];
        controller_state_pub_->msg_.joint_names[i + wheel_joints_size_] = rear_wheel_names[i];
        controller_state_pub_->msg_.joint_names[i + wheel_joints_size_ + 2] = front_steering_names[i];
        controller_state_pub_->msg_.joint_names[i + wheel_joints_size_ + 4] = rear_steering_names[i];
      }

      desired_front_steering_joints_.resize(wheel_joints_size_, 0.0);
      desired_rear_steering_joints_.resize(wheel_joints_size_, 0.0);
      desired_front_wheel_joints_.resize(wheel_joints_size_, 0.0);
      desired_rear_wheel_joints_.resize(wheel_joints_size_, 0.0);
      steering_joints_pub_.resize(4,0.0);

    }


    hardware_interface::VelocityJointInterface *const vel_joint_hw = robot_hw->get<hardware_interface::VelocityJointInterface>();
    hardware_interface::PositionJointInterface *const pos_joint_hw = robot_hw->get<hardware_interface::PositionJointInterface>();

    // Get the joint object to use in the realtime loop
    for (size_t i = 0; i < front_wheel_joints_.size(); ++i)
    {
      ROS_INFO_STREAM_NAMED(name_,
                            "Adding left wheel with joint name: " << front_wheel_names[i]
                            << " and right wheel with joint name: " << rear_wheel_names[i]);
      front_wheel_joints_[i] = vel_joint_hw->getHandle(front_wheel_names[i]);  // throws on failure
      rear_wheel_joints_[i] = vel_joint_hw->getHandle(rear_wheel_names[i]);  // throws on failure
       front_wheel_joints_[0].setCommand(1);

    }

    // Get the steering joint object to use in the realtime loop
    for (size_t i = 0; i < front_steering_joints_.size(); ++i)
    {
      ROS_INFO_STREAM_NAMED(name_,
                            "Adding left steering with joint name: " << front_steering_names[i]
                            << " and right steering with joint name: " << rear_steering_names[i]);
      front_steering_joints_[i] = pos_joint_hw->getHandle(front_steering_names[i]);  // throws on failure
      rear_steering_joints_[i] = pos_joint_hw->getHandle(rear_steering_names[i]);  // throws on failure
    }

    sub_command_ = controller_nh.subscribe("cmd_vel", 1, &FourWheelCarlikeSteeringController::cmdVelCallback, this);
    //sub_command_four_wheel_steering_ = controller_nh.subscribe("cmd_four_wheel_steering", 1, &FourWheelCarlikeSteeringController::cmdFourWheelSteeringCallback, this);
    myPublisher_ = controller_nh.advertise<std_msgs::Float64>("stuff", 1);
    myPublisher1_ = controller_nh.advertise<std_msgs::Float64>("stuff1", 1);
    myPublisher_xvel_ = controller_nh.advertise<std_msgs::Float64>("stuff2", 1);
    myPublisher_yvel_ = controller_nh.advertise<std_msgs::Float64>("stuff3", 1);
    return true;
  }


///////////////////////////////////////////////////////////////////////////////////////////////////
  void FourWheelCarlikeSteeringController::update(const ros::Time& time, const ros::Duration& period)
  {
    //updateOdometry(time);
    updateCommand(time, period);
    publishWheelData(time, period);
  }
/////////////////////////////////////////////////////////////////////////////////
  void FourWheelCarlikeSteeringController::starting(const ros::Time& time)
  {
    brake();

    // Register starting time used to keep fixed rate
    last_state_publish_time_ = time;

    odometry_.init(time);
  }

///////////////////////////////////////////////////////////////////////////////
  void FourWheelCarlikeSteeringController::stopping(const ros::Time& /*time*/)
  {
    brake();
  }


//////////////////////////////////////////////////////////////////////////////////////
  void FourWheelCarlikeSteeringController::updateCommand(const ros::Time& time, const ros::Duration& period)
  {
    // Retreive current velocity command and time step:
    Command* cmd;
    CommandTwist curr_cmd_twist = *(command_twist_.readFromRT());
    Command4ws curr_cmd_4ws = *(command_four_wheel_steering_.readFromRT());



    if(curr_cmd_4ws.stamp >= curr_cmd_twist.stamp)
    {
      cmd = &curr_cmd_4ws;
      enable_twist_cmd_ = false;
    }
    else
    {
      cmd = &curr_cmd_twist;
      enable_twist_cmd_ = true;
    }
    const double dt = (time - cmd->stamp).toSec();
    // Brake if cmd_vel has timeout:
    if (dt > cmd_vel_timeout_)
    {
      curr_cmd_twist.lin_x = 0.0;
      curr_cmd_twist.lin_y = 0.0;
      curr_cmd_twist.ang = 0.0;
      curr_cmd_4ws.lin = 0.0;
      curr_cmd_4ws.front_steering = 0.0;
      curr_cmd_4ws.rear_steering = 0.0;
    }

    const double cmd_dt(period.toSec());



    const double angular_speed = odometry_.getAngular();
    const double steering_track = track_-2*wheel_steering_y_offset_;

    ROS_DEBUG_STREAM("angular_speed "<<angular_speed<< " wheel_radius_ "<<wheel_radius_);
    double vel_left_front = 0, vel_right_front = 0;
    double vel_left_rear = 0, vel_right_rear = 0;
    double front_left_steering = 0, front_right_steering = 0;
    double rear_left_steering = 0, rear_right_steering = 0;

    if(enable_twist_cmd_ == true)
    {
      // Limit velocities and accelerations:
      limiter_lin_.limit(curr_cmd_twist.lin_x, last0_cmd_.lin_x, last1_cmd_.lin_x, cmd_dt);
      limiter_ang_.limit(curr_cmd_twist.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);
      last1_cmd_ = last0_cmd_;
      last0_cmd_ = curr_cmd_twist;


      const double dt = (time - last_update_timestamp_).toSec();


      last_update_timestamp_ = time;

       rtObj.rtU.sampleTime = dt;
      //rtObj.rtU.sampleTime = dtn;

      const double dt_reset = (time - cmd->stamp).toSec();
      rtObj.rtU.reset_sig = reset_signal;

        //printf("reset_sig: %f\n", dt_reset);


/*
      if(dt_reset > 0.2)
      {
        curr_cmd_twist.lin_x =0;
        curr_cmd_twist.lin_y =0;
        curr_cmd_twist.ang =0;
      }

      */


      rtObj.rtU.Ref[0] = curr_cmd_twist.lin_x;
      rtObj.rtU.Ref[1] = curr_cmd_twist.lin_y;
      rtObj.rtU.Ref[2] = curr_cmd_twist.ang;

      rtObj.rtU.MBeta[0] = front_steering_joints_[0].getPosition();
      rtObj.rtU.MBeta[1] = front_steering_joints_[1].getPosition();
      rtObj.rtU.MBeta[2] = rear_steering_joints_[1].getPosition();
      rtObj.rtU.MBeta[3] = rear_steering_joints_[0].getPosition();

      rtObj.rtU.phidot[0] = front_wheel_joints_[0].getVelocity();
      rtObj.rtU.phidot[1] = front_wheel_joints_[1].getVelocity();
      rtObj.rtU.phidot[2] = rear_wheel_joints_[1].getVelocity();
      rtObj.rtU.phidot[3] = rear_wheel_joints_[0].getVelocity();



      rtObj.step();

      const double dtn = (time - cmd->stamp).toSec();

      //printf("dtn: %f\n", dtn);

      /*

      if(dtn > 0.2) //homing position after timeout
      {
          rtObj.rtY.Beta[0]=0;
          rtObj.rtY.Beta[1]=0;
          rtObj.rtY.Beta[3]=0;
          rtObj.rtY.Beta[2]=0;
          rtObj.rtY.Phidot[1]=0;
          rtObj.rtY.Phidot[2]=0;
          rtObj.rtY.Phidot[3]=0;
          rtObj.rtY.Phidot[4]=0;
      }

      */




      // Compute wheels velocities:

      vel_left_front = rtObj.rtY.Phidot[0];
      vel_right_front = rtObj.rtY.Phidot[1];
      vel_left_rear = rtObj.rtY.Phidot[3];
      vel_right_rear = rtObj.rtY.Phidot[2];

      front_left_steering = rtObj.rtY.Beta[0];
      front_right_steering = rtObj.rtY.Beta[1];
      rear_left_steering = rtObj.rtY.Beta[3];
      rear_right_steering = rtObj.rtY.Beta[2];





/*

      //new part: handling the range of the joint controller
      if(front_left_steering>3.14)
      {front_left_steering=front_left_steering-6.28;}
      else
      {
      if(front_left_steering<-3.14)
      {front_left_steering=front_left_steering+6.28;}
      }


      if(front_right_steering>3.14)
      {front_right_steering=front_right_steering-6.28;}
      else {
      if(front_right_steering<-3.14)
      {front_right_steering=front_right_steering+6.28;}
       }


      if(rear_left_steering>3.14)
      {rear_left_steering=rear_left_steering-6.28;}
      else {
      if(rear_left_steering<-3.14)
      {rear_left_steering=rear_left_steering+6.28;}
      }

      if(rear_right_steering>3.14)
      {rear_right_steering=rear_right_steering-6.28;}
      else{
      if(rear_right_steering<-3.14)
      {rear_right_steering=rear_right_steering+6.28;}
      }

    }

*/
/*

      if(curr_cmd_twist.lin_x ==0 && curr_cmd_twist.lin_y ==0 && curr_cmd_twist.ang ==0)
      {
        front_left_steering=0 ;
        front_right_steering=0;
        rear_left_steering=0;
        rear_right_steering=0;
        vel_left_front = 0;
        vel_right_front = 0;
        vel_left_rear = 0;
        vel_right_rear = 0;
        }
*/

/*
      const double dtn = (time - cmd->stamp).toSec();


      if(dtn > 0.5 && curr_cmd_twist.lin_x ==0 && curr_cmd_twist.lin_y ==0 && curr_cmd_twist.ang ==0) //homing position after timeout
      {
          rtObj.rtY.Beta[0]=0;
          rtObj.rtY.Beta[1]=0;
          rtObj.rtY.Beta[3]=0;
          rtObj.rtY.Beta[2]=0;
        front_left_steering=0 ;
        front_right_steering=0;
        rear_left_steering=0;
        rear_right_steering=0;
        vel_left_front = 0;
        vel_right_front = 0;
        vel_left_rear = 0;
        vel_right_rear = 0;
      }

      */

      /////////////////////////////////////////////////////////


      desired_front_steering_joints_[0] = front_left_steering;
      desired_front_steering_joints_[1] = front_right_steering;

      desired_rear_steering_joints_[0] = rear_left_steering;
      desired_rear_steering_joints_[1] = rear_right_steering;

      desired_front_wheel_joints_[0] = vel_left_front;
      desired_front_wheel_joints_[1] = vel_right_front;

      desired_rear_wheel_joints_[0] = vel_left_rear;
      desired_rear_wheel_joints_[1] = vel_right_rear;

      steering_joints_pub_[0] = front_left_steering;
      steering_joints_pub_[1] = front_right_steering;

      steering_joints_pub_[2] = rear_left_steering;
      steering_joints_pub_[3] = rear_right_steering;


      std_msgs::Float64 test;
      std_msgs::Float64 test1;
      std_msgs::Float64 xvel;
      std_msgs::Float64 yvel;



      double xdot = rtObj.rtY.Vel[0];
      double ydot = rtObj.rtY.Vel[1];
      double thetdot = rtObj.rtY.Vel[2];

      PoseTheta=PoseTheta+thetdot*dt;
      //printf("this is the posetheta: %f\n", PoseTheta);
      //PoseTheta=PoseTheta+thetdot*cmd_dt;
       PoseX = rtObj.rtY.Pose[0];
       PoseY = rtObj.rtY.Pose[1];
       PoseTheta = rtObj.rtY.Pose[2];


       //coming back in odom frame
      double v_wx = xdot * cos(PoseTheta) - ydot * sin(PoseTheta);
      double v_wy = xdot * sin(PoseTheta) + ydot * cos(PoseTheta);

      //PoseX=PoseX+v_wx*dt;
      //PoseY=PoseY+v_wy*dt;

      //PoseX=PoseX+xdot*dt;
      //PoseY=PoseY+ydot*dt;

    //  printf("\n this is the heading: %f \n", PoseTheta);


      //printf("this is PoseX: %f\n",PoseX);


      // Publish odometry message
      if (last_state_publish_time_ + publish_period_ < time)
      {
        last_state_publish_time_ += publish_period_;

        const geometry_msgs::Quaternion orientation(
              tf::createQuaternionMsgFromYaw(PoseTheta));

        // Populate odom message and publish

        if (odom_pub_->trylock())
        { //printf("\ntest test lock \n" );
          odom_pub_->msg_.header.stamp = time;
          odom_pub_->msg_.pose.pose.position.x = PoseX;
          odom_pub_->msg_.pose.pose.position.y = PoseY;
          odom_pub_->msg_.pose.pose.orientation = orientation;
        //  odom_pub_->msg_.pose.pose.orientation.yaw_deg = PoseTheta ;
          odom_pub_->msg_.twist.twist.linear.x  = xdot;
          odom_pub_->msg_.twist.twist.linear.y  = ydot;
          odom_pub_->msg_.twist.twist.angular.z = thetdot;
          odom_pub_->unlockAndPublish();
        }
        // Publish tf /odom frame
        if (enable_odom_tf_ && tf_odom_pub_->trylock())
        {
          geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
          odom_frame.header.stamp = time;
          odom_frame.transform.translation.x = PoseX;
          odom_frame.transform.translation.y = PoseY;
          odom_frame.transform.rotation = orientation;
          tf_odom_pub_->unlockAndPublish();
        }
      }

    ROS_DEBUG_STREAM_THROTTLE(1, "vel_left_rear "<<vel_left_rear<<" front_right_steering "<<front_right_steering);
    // Set wheels velocities:
    if(front_wheel_joints_.size() == 2 && rear_wheel_joints_.size() == 2)
    {
      front_wheel_joints_[0].setCommand(vel_left_front);
      front_wheel_joints_[1].setCommand(vel_right_front);
      rear_wheel_joints_[0].setCommand(vel_left_rear);
      rear_wheel_joints_[1].setCommand(vel_right_rear);
    }

    /// TODO check limits to not apply the same steering on right and left when saturated !
    if(front_steering_joints_.size() == 2 && rear_steering_joints_.size() == 2)
    {
      front_steering_joints_[0].setCommand(front_left_steering);
      front_steering_joints_[1].setCommand(front_right_steering);
      rear_steering_joints_[0].setCommand(rear_left_steering);
      rear_steering_joints_[1].setCommand(rear_right_steering);
    }
   }
  }




/////////////////////////////////////////////////////////////////////////////////////
  void FourWheelCarlikeSteeringController::brake()
  {
    const double vel = 0.0;
    for (size_t i = 0; i < front_wheel_joints_.size(); ++i)
    {
      front_wheel_joints_[i].setCommand(vel);
      rear_wheel_joints_[i].setCommand(vel);
    }

    const double pos = 0.0;
    for (size_t i = 0; i < front_steering_joints_.size(); ++i)
    {
      front_steering_joints_[i].setCommand(pos);
      rear_steering_joints_[i].setCommand(pos);
    }
  }



/////////////////////////////////////////////////////////////////////////////////////////////
  void FourWheelCarlikeSteeringController::cmdVelCallback(const geometry_msgs::Twist& command)
  {
    if (isRunning())
    {
      if(std::isnan(command.angular.z) || std::isnan(command.linear.x))
      {
        ROS_WARN("Received NaN in geometry_msgs::Twist. Ignoring command.");
        return;
      }
      command_struct_twist_.ang   = command.angular.z;
      command_struct_twist_.lin_x   = command.linear.x;
      command_struct_twist_.lin_y   = command.linear.y;
      command_struct_twist_.stamp = ros::Time::now();
      command_twist_.writeFromNonRT (command_struct_twist_);
      ROS_DEBUG_STREAM_NAMED(name_,
                             "Added values to command. "
                             << "Ang: "   << command_struct_twist_.ang << ", "
                             << "Lin x: " << command_struct_twist_.lin_x << ", "
                             << "Lin y: " << command_struct_twist_.lin_y << ", "
                             << "Stamp: " << command_struct_twist_.stamp);
    }
    else
    {
      ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    }
  }




/////////////////////////////////////////////////////////////////////////////////////////
  bool FourWheelCarlikeSteeringController::getWheelNames(ros::NodeHandle& controller_nh,
                              const std::string& wheel_param,
                              std::vector<std::string>& wheel_names)
  {
      XmlRpc::XmlRpcValue wheel_list;
      if (!controller_nh.getParam(wheel_param, wheel_list))
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Couldn't retrieve wheel param '" << wheel_param << "'.");
        return false;
      }

      if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        if (wheel_list.size() == 0)
        {
          ROS_ERROR_STREAM_NAMED(name_,
              "Wheel param '" << wheel_param << "' is an empty list");
          return false;
        }

        for (int i = 0; i < wheel_list.size(); ++i)
        {
          if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
          {
            ROS_ERROR_STREAM_NAMED(name_,
                "Wheel param '" << wheel_param << "' #" << i <<
                " isn't a string.");
            return false;
          }
        }

        wheel_names.resize(wheel_list.size());
        for (int i = 0; i < wheel_list.size(); ++i)
        {
          wheel_names[i] = static_cast<std::string>(wheel_list[i]);
          //ROS_INFO_STREAM("wheel name "<<i<<" " << wheel_names[i]);
        }
      }
      else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        wheel_names.push_back(wheel_list);
      }
      else
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Wheel param '" << wheel_param <<
            "' is neither a list of strings nor a string.");
        return false;
      }
      return true;
  }


//////////////////////////////////////////////////////////////////////////////////////////////////////////
  void FourWheelCarlikeSteeringController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
  {
    // Get and check params for covariances
    XmlRpc::XmlRpcValue pose_cov_list;
    controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
    ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_cov_list.size() == 6);
    for (int i = 0; i < pose_cov_list.size(); ++i)
      ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    XmlRpc::XmlRpcValue twist_cov_list;
    controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i)
      ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom_test", 100));
    odom_pub_->msg_.header.frame_id = "odom";
    odom_pub_->msg_.child_frame_id = base_frame_id_;
    odom_pub_->msg_.pose.pose.position.z = 0;
    odom_pub_->msg_.pose.covariance = {
        static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0.,
        0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0.,
        0., 0., static_cast<double>(pose_cov_list[2]), 0., 0., 0.,
        0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0.,
        0., 0., 0., 0., static_cast<double>(pose_cov_list[4]), 0.,
        0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5]) };
    odom_pub_->msg_.twist.twist.linear.y  = 0;
    odom_pub_->msg_.twist.twist.linear.z  = 0;
    odom_pub_->msg_.twist.twist.angular.x = 0;
    odom_pub_->msg_.twist.twist.angular.y = 0;
    odom_pub_->msg_.twist.covariance = {
        static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0.,
        0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0.,
        0., 0., static_cast<double>(twist_cov_list[2]), 0., 0., 0.,
        0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0.,
        0., 0., 0., 0., static_cast<double>(twist_cov_list[4]), 0.,
        0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5]) };
    //odom_4ws_pub_.reset(new realtime_tools::RealtimePublisher<four_wheel_steering_msgs::FourWheelSteeringStamped>(controller_nh, "odom_steer", 100));
    //odom_4ws_pub_->msg_.header.frame_id = "odom";

    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf_test", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";
  }





////////////////////////////////////////////////////////////////////////////////
  void FourWheelCarlikeSteeringController::publishWheelData(const ros::Time& time, const ros::Duration& period)
  {
    if (publish_wheel_joint_controller_state_ && controller_state_pub_->trylock())
    {
      const double cmd_dt(period.toSec());
      for (size_t i = 0; i < wheel_joints_size_; ++i)
      {
        // Actual
        controller_state_pub_->msg_.actual.positions[i]     = front_wheel_joints_[i].getPosition();
        controller_state_pub_->msg_.actual.positions[i+ wheel_joints_size_] = rear_wheel_joints_[i].getPosition();
        controller_state_pub_->msg_.actual.positions[i+ wheel_joints_size_+2] = front_steering_joints_[i].getPosition();
        controller_state_pub_->msg_.actual.positions[i+ wheel_joints_size_+4] = rear_steering_joints_[i].getPosition();

        controller_state_pub_->msg_.actual.velocities[i]     = front_wheel_joints_[i].getVelocity();
        controller_state_pub_->msg_.actual.velocities[i+ wheel_joints_size_] = rear_wheel_joints_[i].getVelocity();
        controller_state_pub_->msg_.actual.velocities[i+ wheel_joints_size_+2] = front_steering_joints_[i].getVelocity();
        controller_state_pub_->msg_.actual.velocities[i+ wheel_joints_size_+4] = rear_steering_joints_[i].getVelocity();

        // Desired
        controller_state_pub_->msg_.desired.positions[i]     = front_wheel_joints_[i].getPosition();
        controller_state_pub_->msg_.desired.positions[i+ wheel_joints_size_] = rear_wheel_joints_[i].getPosition();
        controller_state_pub_->msg_.desired.positions[i+ wheel_joints_size_+2] = desired_front_steering_joints_[i];
        controller_state_pub_->msg_.desired.positions[i+ wheel_joints_size_+4] = desired_rear_steering_joints_[i];

        controller_state_pub_->msg_.desired.velocities[i]     = desired_front_wheel_joints_[i];
        controller_state_pub_->msg_.desired.velocities[i+ wheel_joints_size_] = desired_rear_wheel_joints_[i];
        controller_state_pub_->msg_.desired.velocities[i+ wheel_joints_size_+2] = front_steering_joints_[i].getVelocity();
        controller_state_pub_->msg_.desired.velocities[i+ wheel_joints_size_+4] = rear_steering_joints_[i].getVelocity();
      }

      controller_state_pub_->unlockAndPublish();
    }
  }

} // namespace four_wheel_carlike_steering_controller
