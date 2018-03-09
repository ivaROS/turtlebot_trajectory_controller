/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file /kobuki_controller_tutorial/include/kobuki_controller_tutorial/bump_blink_controller.hpp
 *
 * @brief "Bump-Blink"-controller for the Kobuki controller tutorial
 *
 * A simple nodelet-based controller for Kobuki, which makes one of Kobuki's LEDs blink, when a bumper is pressed.
 *
 * @author Marcus Liebhardt, Yujin Robot
 **/


/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include "trajectory_controller.h"
#include <tf2_pips/tf2_trajectory.h>
#include <turtlebot_trajectory_controller/TurtlebotControllerConfig.h>

#include <dynamic_reconfigure/server.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <yocs_controllers/default_controller.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <nav_msgs/Odometry.h>
#include <trajectory_generator_ros_interface.h>
#include <tf/transform_datatypes.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>

#include <memory>




namespace kobuki
{

/**
 * @ brief A simple bump-blink-controller
 *
 * A simple nodelet-based controller for Kobuki, which makes one of Kobuki's LEDs blink, when a bumper is pressed.
 */

  TrajectoryController::TrajectoryController(ros::NodeHandle& nh, ros::NodeHandle& pnh) : 
    Controller(), 
    nh_(nh), 
    pnh_(pnh, name_)
  { };
  


  /**
   * Set-up necessary publishers/subscribers
   * @return true, if successful
   */
  bool TrajectoryController::init()
  {
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(); //optional parameter: ros::Duration(cache time) (default=10) (though it doesn't seem to accept it!)
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    
    reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh_);
    reconfigure_server_->setCallback(boost::bind(&TrajectoryController::configCB, this, _1, _2));
    
    setupParams();
    setupPublishersSubscribers();

    
    
    this->enable();
    curr_index_ = -1;
    executing_ = false;
    
    return true;
  };  
  
  void TrajectoryController::setupPublishersSubscribers()
  {
    ROS_DEBUG_NAMED(name_, "Setup publishers and subscribers");
    
    enable_controller_subscriber_ = nh_.subscribe("enable", 10, &TrajectoryController::enableCB, this);
    disable_controller_subscriber_ = nh_.subscribe("disable", 10, &TrajectoryController::disableCB, this);
    
    //Not sure if it is good idea to use the spinner: only one asyncspinner can run in a process, and when using nodelets that is very dangerous assumption
    if(use_odom_spinner_)
    {
      ROS_WARN("Using spinner");
      odom_nh_.setCallbackQueue(&odom_queue_);
      odom_subscriber_ = odom_nh_.subscribe("odom", 1, &TrajectoryController::OdomCB, this);
      odom_spinner_ = std::make_shared<ros::AsyncSpinner>(0, &odom_queue_); //1 is for the number of threads
      odom_spinner_->start();
    }
    else
    {
      odom_subscriber_ = nh_.subscribe("odom", 1, &TrajectoryController::OdomCB, this);
    }
    /*Should the queue be 1 or 2? I really only want to act on the most recent data. However, if a queue of 1 means that a new message
    won't be stored if it arrives while the previous is being processed, but could otherwise be processed before the next comes, then
    2 would be better. But if 2 means that older info will be used while newer info is already waiting in the queue, then should use 1 */
    
    tf_filter_.reset(new tf_filter(trajectory_subscriber_, *tfBuffer_, odom_frame_id_, trajectory_queue_size_, nh_));
    trajectory_subscriber_.subscribe(nh_, "desired_trajectory", trajectory_queue_size_);
    tf_filter_->registerCallback(boost::bind(&TrajectoryController::TrajectoryCB, this, _1));
    tf_filter_->setTolerance(ros::Duration(0.01));

    command_publisher_ = nh_.advertise< geometry_msgs::Twist >("cmd_vel_mux/input/navi", 1);
    trajectory_odom_publisher_ = nh_.advertise< nav_msgs::Odometry >("desired_odom", 10);
    transformed_trajectory_publisher_ = nh_.advertise< pips_trajectory_msgs::trajectory_points >("transformed_trajectory", 10);
  }
  
  void TrajectoryController::configCB(turtlebot_trajectory_controller::TurtlebotControllerConfig &config, uint32_t level) {
    ROS_INFO_STREAM_NAMED(name_, "Reconfigure Request:\n\tk_turn =\t" << config.k_turn << "\n\tk_drive_x =\t"<< config.k_drive_x <<"\n\tk_drive_y =\t" << config.k_drive_y);
    k_turn_ = config.k_turn;
    k_drive_x_ = config.k_drive_x;
    k_drive_y_ = config.k_drive_y;
/*
max_ang_v = config.max_ang_v;
max_ang_acc = config.max_ang_acc;
max_lin_v = config.max_lin_v;
max_lin_acc = config.max_lin_acc;
    */
  }
  
  void TrajectoryController::setupParams()
  {
    ROS_DEBUG_NAMED(name_, "Setup parameters");
   /* 
    pnh_.param<std::string>("odom_param_name", k_drive_, 1.0);
    pnh_.param<std::string>("k_turn", k_turn_, 1.0);
    */
    nh_.param<std::string>("/mobile_base/odom_frame", odom_frame_id_, "odom");
    nh_.param<std::string>("/mobile_base/base_frame", base_frame_id_, "base_footprint");

    pnh_.param<bool>("odom_spinner", use_odom_spinner_, false);
    
    pnh_.setParam("odom_spinner", use_odom_spinner_);


  }
  
 
void TrajectoryController::enableCB(const std_msgs::Empty::ConstPtr& msg)
{
  if (this->enable())
  {
    ROS_INFO_NAMED(name_, "Controller has been enabled.");
    start_time_ = ros::Time::now();
  }
  else
  {
    ROS_DEBUG_NAMED(name_, "Controller was already enabled.");
  }
};

void TrajectoryController::disableCB(const std_msgs::Empty::ConstPtr& msg)
{
  if (this->disable())
  {
    ROS_INFO_NAMED(name_, "Controller has been disabled.");
    stop();
  }
  else
  {
    ROS_DEBUG_NAMED(name_, "Controller was already disabled.");
  }
};

void TrajectoryController::stop(bool force_stop)
{
  if(executing_)
  {
    executing_ = false;
    curr_index_ = -1;
    ROS_WARN_NAMED(name_, "Interrupted trajectory.");
  }
  if(force_stop)
  {
    geometry_msgs::Twist::ConstPtr command(new geometry_msgs::Twist);
    command_publisher_.publish(command);
  }
}


void TrajectoryController::TrajectoryCB(const pips_trajectory_msgs::trajectory_points::ConstPtr& msg)
{

  ROS_DEBUG_NAMED(name_, "Trajectory received.");
  ROS_INFO_STREAM("RECEIVED TRAJECTORY");
  if (this->getState())
  {
    if(executing_)  //TODO: maybe change this to be a conditional log statement
    {
      ROS_DEBUG_NAMED(name_, "Preempting previous trajectory");
    }
    

    try
    {
      //Lock trajectory mutex while updating trajectory. Ensures that the desired state is computed using only 1 trajectory.
      {
        boost::mutex::scoped_lock lock(trajectory_mutex_);
      
        desired_trajectory_ = tfBuffer_->transform(*msg, odom_frame_id_);  // Uses the time and frame provided by header of msg (tf2_ros::buffer_interface.h)
        curr_index_ = 0;
        executing_ = true;
      }
      
      ROS_DEBUG_STREAM_THROTTLE_NAMED(5, name_, "Successfully transformed trajectory from '" << msg->header.frame_id << "' to '" << odom_frame_id_);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE_NAMED(5, name_, "Unable to execute trajectory: %s",ex.what());
        return;
    }
      
      
    transformed_trajectory_publisher_.publish(desired_trajectory_);
    ROS_DEBUG_STREAM_NAMED(name_, "Preparing to execute.");
    

  }
  else
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Controller disabled, will not execute trajectory. [" << name_ <<"]");
  }

}


void TrajectoryController::OdomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  curr_odom_ = msg;
  
  odom_rate.addTime(msg->header);
  
  // TODO: move the rate logging into the rate tracker, allowing the messages to be enabled separately
  ROS_DEBUG_STREAM_THROTTLE_NAMED(2, name_,"Odom rate: " << odom_rate.getRate() << " (" << odom_rate.getNumSamples() << " samples). Current delay: " << odom_rate.getLastDelay() << "s; Average delay: " << odom_rate.getAverageDelay() << "s.");
  
  
  if (this->getState() && executing_) // check, if the controller is active
  {

    ROS_DEBUG_STREAM_NAMED(name_, "Odom@ " << msg->header.stamp << "s: (" << msg->pose.pose.position.x << "," << msg->pose.pose.position.y << ") and " << msg->pose.pose.orientation.w <<"," << msg->pose.pose.orientation.z);
  
    const nav_msgs::Odometry::ConstPtr desired = TrajectoryController::getDesiredState(msg->header);
    trajectory_odom_publisher_.publish(desired);
    
    geometry_msgs::Twist::ConstPtr command = TrajectoryController::ControlLaw(msg, desired);
    command_publisher_.publish(command);
    ROS_DEBUG_STREAM_NAMED(name_, "Command: " << command->linear.x <<"m/s, " << command->angular.z << "rad/s");

  }

}





Eigen::Matrix2cd TrajectoryController::getComplexMatrix(double x, double y, double quat_w, double quat_z)
{
  std::complex<double> phase(quat_w, quat_z);
  phase = phase*phase;
  
  Eigen::Matrix2cd g(2,2);
  //g.real()(0,0) = phase.real();
  g.real()(0,1) = x;
  g.real()(1,0) = 0;
  g.real()(1,1) = 1;

  //g.imag()(0,0) = phase.imag();
  g.imag()(0,1) = y;
  g.imag()(1,0) = 0;
  g.imag()(1,1) = 0;
  
  g(0,0) = phase;

  return g;
}



geometry_msgs::Twist::ConstPtr TrajectoryController::ControlLaw(const nav_msgs::Odometry::ConstPtr& current, const nav_msgs::Odometry::ConstPtr& desired)
{
    geometry_msgs::Point position = current->pose.pose.position;
    geometry_msgs::Quaternion orientation = current->pose.pose.orientation;

    Eigen::Matrix2cd g_curr = TrajectoryController::getComplexMatrix(position.x, position.y, orientation.w, orientation.z);
    ROS_DEBUG_NAMED(name_, "[%s] Current:\n%f + %fi, %f + %fi\n%f + %fi, %f + %fi", name_.c_str(), g_curr.real()(0,0), g_curr.imag()(0,0), g_curr.real()(0,1), g_curr.imag()(0,1), g_curr.real()(1,0), g_curr.imag()(1,0), g_curr.real()(1,1), g_curr.imag()(1,1));

    position = desired->pose.pose.position;
    orientation = desired->pose.pose.orientation;

    Eigen::Matrix2cd g_des = TrajectoryController::getComplexMatrix(position.x, position.y, orientation.w, orientation.z);
    ROS_DEBUG_NAMED(name_, "[%s] Desired:\n%f + %fi, %f + %fi\n%f + %fi, %f + %fi", name_.c_str(), g_des.real()(0,0), g_des.imag()(0,0), g_des.real()(0,1), g_des.imag()(0,1), g_des.real()(1,0), g_des.imag()(1,0), g_des.real()(1,1), g_des.imag()(1,1));

    Eigen::Matrix2cd g_error = g_curr.inverse() * g_des;

    ROS_DEBUG_NAMED(name_, "[%s] Error:\n%f + %fi, %f + %fi\n%f + %fi, %f + %fi", name_.c_str(), g_error.real()(0,0), g_error.imag()(0,0), g_error.real()(0,1), g_error.imag()(0,1), g_error.real()(1,0), g_error.imag()(1,0), g_error.real()(1,1), g_error.imag()(1,1));

    
    double theta_error = std::arg(g_error(0,0));
    double x_error = g_error.real()(0,1);
    double y_error = g_error.imag()(0,1);
    
    double v_ang_fb = theta_error * k_turn_ + y_error*k_drive_y_;
    double v_lin_fb = x_error * k_drive_x_;

    double v_ang_ff = desired->twist.twist.angular.z;
    double v_lin_ff = desired->twist.twist.linear.x;

    double v_ang = v_ang_fb + v_ang_ff;
    double v_lin = v_lin_fb + v_lin_ff;
    
    /*
      Saturation/limits:
      Velocity limits: simply constrain v_ang and v_lin to be within defined limits.
      Acceleration limits: possibly limit the magnitude of velocity difference between current vel and desired
      Note, however, that the yocs_velocity_smoother package already applies robot velocity/acceleration limits. I am not sure whether it is being used at the moment, or if it is adjustable, or if additional limits are needed here. 
    
    Update: yocs_velocity_smoother is not used by default, and it is adjustable
    
    */
/*
double delta_t = .05;
double max_ang_v = 4;//5;
double max_ang_acc = 1.78;
double max_lin_v = .5;//.7;
double max_lin_acc = .55;


    v_lin = near_identity::saturate(v_lin, 0, max_lin_v);
    v_ang = near_identity::saturate(v_ang, -max_ang_v, max_ang_v);
*/
	//Simple thresholding, likely unnecessary
	if(v_lin > .5) v_lin = .5;
	if(v_ang > 4) v_ang = 4;
	if(v_ang < -4) v_ang = -4;

/*

    double ang_accel = (v_ang - current->twist.twist.angular.z)/ delta_t;
    ang_accel = near_identity::applyLimits(ang_accel, v_ang, -max_ang_v, max_ang_v,  -max_ang_acc, max_ang_acc);

    v_ang = current->twist.twist.angular.z + ang_accel * delta_t;

    double lin_accel = (v_lin - current->twist.twist.linear.x)/delta_t;
    lin_accel = near_identity::applyLimits(lin_accel, v_lin, -max_lin_v, max_lin_v, -max_lin_acc, max_lin_acc);

    v_lin = current->twist.twist.linear.x + lin_accel * delta_t;
*/
    geometry_msgs::Vector3 linear;
    linear.x = v_lin;

    geometry_msgs::Vector3 angular;
    angular.z = v_ang;

    geometry_msgs::Twist::Ptr command(new geometry_msgs::Twist);
    command->linear = linear;
    command->angular = angular;
    
    geometry_msgs::Twist::ConstPtr const_command = command;

    ROS_DEBUG_STREAM_NAMED(name_, "Linear Error: " << x_error << "m, Angular Error: " << theta_error << "rad");
    
    return const_command;
}

// TODO: look into trying to follow the trajectory more as a function of position than of time;
// Possibly rerun near_identity;
nav_msgs::OdometryPtr TrajectoryController::getDesiredState(const std_msgs::Header& header)
{
  nav_msgs::OdometryPtr odom(new nav_msgs::Odometry);
  pips_trajectory_msgs::trajectory_point pre_point, post_point;
  double pre_time_fraction = 1;
  double t;
  std::size_t num_points;
  int post_index;
  
  {
    boost::mutex::scoped_lock lock(trajectory_mutex_);  //While computing desired state, don't change the trajectory
    
    num_points = desired_trajectory_.points.size();
    odom->header.frame_id = desired_trajectory_.header.frame_id;
  
    ros::Duration elapsed_time = header.stamp - desired_trajectory_.header.stamp;
    t = elapsed_time.toSec();
  
    //This 'should' update curr_index to refer to the last trajectory point before the desired time.
    for(; curr_index_ < num_points -1 && desired_trajectory_.points[curr_index_+1].time < elapsed_time; curr_index_++);
  
   
    //This handles the case where we've reached the end of the trajectory time
    post_index = std::min(curr_index_+1, num_points-1);
    
   
    pre_point = desired_trajectory_.points[curr_index_];
    post_point = desired_trajectory_.points[post_index];
    
    ros::Duration pre_time = elapsed_time - pre_point.time;
    ros::Duration period = post_point.time - pre_point.time;
    
    
    if(curr_index_ < desired_trajectory_.points.size()-1)
    {
      pre_time_fraction = pre_time.toSec()/period.toSec();
    }
    
    if(curr_index_ == desired_trajectory_.points.size()-1)
    {
        executing_ = false;
        curr_index_ = -1;
    }
    
  }

  double x = pre_point.x*(1-pre_time_fraction) + post_point.x*pre_time_fraction;
  double y = pre_point.y*(1-pre_time_fraction) + post_point.y*pre_time_fraction;;
  double theta = pre_point.theta*(1-pre_time_fraction) + post_point.theta*pre_time_fraction;
  double v = pre_point.v*(1-pre_time_fraction) + post_point.v*pre_time_fraction;
  double w = pre_point.w*(1-pre_time_fraction) + post_point.w*pre_time_fraction;

  //This was in the trajectory generator class.
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, theta);


  // Header
  odom->header.stamp = header.stamp;

  // Position
  odom->pose.pose.position.x = x;
  odom->pose.pose.position.y = y;
  odom->pose.pose.position.z = 0.0;
  odom->pose.pose.orientation = quat;

  // Velocity
  odom->twist.twist.linear.x = v;
  odom->twist.twist.linear.y = 0;
  odom->twist.twist.angular.z = w;

  ROS_DEBUG_STREAM_NAMED(name_, "Index: " << curr_index_ << "; # points: " << num_points); 
  ROS_DEBUG_STREAM_NAMED(name_, "Preindex: " << curr_index_ << "; postindex: " << post_index);  
  ROS_DEBUG_STREAM_NAMED(name_, "Desired@ " << t << "s: (" << x << "," << y << ") and " << quat.w <<"," << quat.z);
  


  return odom;
};

} // namespace kobuki
// %EndTag(FULLTEXT)%

