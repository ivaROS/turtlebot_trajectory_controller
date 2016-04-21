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
#include <tf2_trajectory.h>

#include <memory>




namespace kobuki
{

/**
 * @ brief A simple bump-blink-controller
 *
 * A simple nodelet-based controller for Kobuki, which makes one of Kobuki's LEDs blink, when a bumper is pressed.
 */

  TrajectoryController::TrajectoryController(ros::NodeHandle& nh, std::string& name) : Controller(), nh_(nh), name_(name){};


  /**
   * Set-up necessary publishers/subscribers
   * @return true, if successful
   */
  bool TrajectoryController::init()
  {
    //Need to delete these on shutdown; or use use shared pointers
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(); //optional parameter: ros::Duration(cache time) (default=10)
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    
    setupParams();
    setupPublishersSubscribers();

    
    
    this->enable();
    curr_index_ = -1;
    executing_ = false;
    
    return true;
  };  
  
  void TrajectoryController::setupPublishersSubscribers()
  {
    ROS_DEBUG_NAMED(private_name_, "Setup publishers and subscribers");
    
    enable_controller_subscriber_ = nh_.subscribe("enable", 10, &TrajectoryController::enableCB, this);
    disable_controller_subscriber_ = nh_.subscribe("disable", 10, &TrajectoryController::disableCB, this);
    
    odom_nh_.setCallbackQueue(&odom_queue_);
    
    /*Should the queue be 1 or 2? I really only want to act on the most recent data. However, if a queue of 1 means that a new message
    won't be stored if it arrives while the previous is being processed, but could otherwise be processed before the next comes, then
    2 would be better. But if 2 means that older info will be used while newer info is already waiting in the queue, then should use 1 */
    
    //Also, it is not a good idea to use the spinner: only one asyncspinner can run in a process, and when using nodelets that is very dangerous assumption
    odom_subscriber_ = odom_nh_.subscribe("/odom", 2, &TrajectoryController::OdomCB, this);
    odom_spinner_ = std::make_shared<ros::AsyncSpinner>(0, &odom_queue_); //1 is for the number of threads
    odom_spinner_->start();
    
    trajectory_subscriber_ = nh_.subscribe("/desired_trajectory", 1, &TrajectoryController::TrajectoryCB, this);

    command_publisher_ = nh_.advertise< geometry_msgs::Twist >("/cmd_vel_mux/input/navi", 10);
    trajectory_odom_publisher_ = nh_.advertise< nav_msgs::Odometry >("/desired_odom", 10);
    transformed_trajectory_publisher_ = nh_.advertise< trajectory_generator::trajectory_points >("/transformed_trajectory", 10);
  }
  
  void TrajectoryController::setupParams()
  {
    ROS_DEBUG_NAMED(private_name_, "Setup parameters");
    
    nh_.param<std::string>("/mobile_base/odom_frame", odom_frame_id_, "odom");
    nh_.param<std::string>("/mobile_base/base_frame", base_frame_id_, "base_footprint");
    nh_.param<double>("k_drive", k_drive_, 1.0);
    nh_.param<double>("k_turn", k_turn_, 1.0);

  }

  
void TrajectoryController::enableCB(const std_msgs::Empty::ConstPtr& msg)
{
  if (this->enable())
  {
    ROS_INFO_NAMED(private_name_, "Controller has been enabled.");
    start_time_ = ros::Time::now();
  }
  else
  {
    ROS_INFO_NAMED(private_name_, "Controller was already enabled.");
  }
};

void TrajectoryController::disableCB(const std_msgs::Empty::ConstPtr& msg)
{
  if (this->disable())
  {
    ROS_INFO_NAMED(private_name_, "Controller has been disabled.");
    if(executing_)
    {
      executing_ = false;
      curr_index_ = -1;
      ROS_WARN_NAMED(private_name_, "Interrupted trajectory.");
    }
  }
  else
  {
    ROS_DEBUG_NAMED(private_name_, "Controller was already disabled.");
  }
};


//This should also use ConstPtr
/*May need to use message filter to ensure transform available;
#include <message_filter.h>
message_filters::Subscriber<MessageType> sub(node_handle_, "topic", 10);
tf::MessageFilter<MessageType> tf_filter(sub, tf_listener_, "/map", 10);
tf_filter.registerCallback(&MyClass::myCallback, this);
*/
void TrajectoryController::TrajectoryCB(const trajectory_generator::trajectory_points msg)
{

  ROS_DEBUG_NAMED(private_name_, "Trajectory received.");
  if (this->getState())
  {
    if(executing_)
    {
      ROS_DEBUG_NAMED(private_name_, "Preempting previous trajectory");
    }
    

    try
    {
      //Lock trajectory mutex while updating trajectory
      {
        boost::mutex::scoped_lock lock(trajectory_mutex_);
      
        desired_trajectory_ = tfBuffer_->transform(msg, odom_frame_id_);  // Uses the time and frame provided by header of msg (tf2_ros::buffer_interface.h)
        curr_index_ = 0;
        executing_ = true;
      }
      
      ROS_DEBUG_STREAM_NAMED(private_name_, "Successfully transformed trajectory from '" << msg.header.frame_id << "' to '" << odom_frame_id_);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN_NAMED(private_name_, "Unable to execute trajectory: %s",ex.what());
        return;
    }
      
      
    transformed_trajectory_publisher_.publish(desired_trajectory_);
    ROS_DEBUG_STREAM_NAMED(private_name_, "Preparing to execute.");
    

  }
  else
  {
    ROS_DEBUG_STREAM_NAMED(private_name_, "Controller disabled, will not execute trajectory. [" << name_ <<"]");
  }

}


void TrajectoryController::OdomCB(const nav_msgs::Odometry::ConstPtr msg)
{
  {
    boost::mutex::scoped_lock lock(odom_mutex_);
    curr_odom_ = msg;
  }
  
  if (this->getState() && executing_) // check, if the controller is active
  {
    ROS_DEBUG_STREAM_NAMED(private_name_, "Odom@ " << msg->header.stamp << "s: (" << msg->pose.pose.position.x << "," << msg->pose.pose.position.y << ") and " << msg->pose.pose.orientation.w <<"," << msg->pose.pose.orientation.z);
  
    const nav_msgs::Odometry::ConstPtr desired = TrajectoryController::getDesiredState(msg->header);
    trajectory_odom_publisher_.publish(desired);
    
    geometry_msgs::Twist::ConstPtr command = TrajectoryController::ControlLaw(msg, desired);
    command_publisher_.publish(command);
    ROS_DEBUG_STREAM_NAMED(private_name_, "Command: " << command->linear.x <<"m/s, " << command->angular.z << "rad/s");
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
    
    double v_ang_fb = theta_error * k_turn_;
    double v_lin_fb = x_error * k_drive_;

    double v_ang_ff = desired->twist.twist.angular.z;
    double v_lin_ff = desired->twist.twist.linear.x;

    double v_ang = v_ang_fb + v_ang_ff;
    double v_lin = v_lin_fb + v_lin_ff;
    
    /*
      Saturation/limits:
      Velocity limits: simply constrain v_ang and v_lin to be within defined limits.
      Acceleration limits: possibly limit the magnitude of velocity difference between current vel and desired
      Note, however, that the yocs_velocity_smoother package already applies robot velocity/acceleration limits. I am not sure whether it is being used at the moment, or if it is adjustable, or if additional limits are needed here. 
    
    
    */

    geometry_msgs::Vector3 linear;
    linear.x = v_lin;

    geometry_msgs::Vector3 angular;
    angular.z = v_ang;

    geometry_msgs::Twist::Ptr command(new geometry_msgs::Twist);
    command->linear = linear;
    command->angular = angular;
    
    geometry_msgs::Twist::ConstPtr const_command = command;

    ROS_DEBUG_STREAM_NAMED(private_name_, "Linear Error: " << x_error << "m, Angular Error: " << theta_error << "rad");
    
    return const_command;
}

nav_msgs::OdometryPtr TrajectoryController::getDesiredState(const std_msgs::Header& header)
{
  nav_msgs::OdometryPtr odom(new nav_msgs::Odometry);
  trajectory_generator::trajectory_point pre_point, post_point;
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

  ROS_DEBUG_STREAM_NAMED(private_name_, "Index: " << curr_index_ << "; # points: " << num_points); 
  ROS_DEBUG_STREAM_NAMED(private_name_, "Preindex: " << curr_index_ << "; postindex: " << post_index);  
  ROS_DEBUG_STREAM_NAMED(private_name_, "Desired@ " << t << "s: (" << x << "," << y << ") and " << quat.w <<"," << quat.z);
  


  return odom;
};

} // namespace kobuki
// %EndTag(FULLTEXT)%

