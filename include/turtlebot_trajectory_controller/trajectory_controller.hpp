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
** Ifdefs
*****************************************************************************/

#ifndef TRAJECTORY_CONTROLLER_HPP_
#define TRAJECTORY_CONTROLLER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
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


namespace kobuki
{

/**
 * @ brief A simple bump-blink-controller
 *
 * A simple nodelet-based controller for Kobuki, which makes one of Kobuki's LEDs blink, when a bumper is pressed.
 */
class TrajectoryController : public yocs::Controller
{
public:
  TrajectoryController(ros::NodeHandle& nh, std::string& name) : Controller(), nh_(nh), name_(name){};
  ~TrajectoryController(){};

  /**
   * Set-up necessary publishers/subscribers
   * @return true, if successful
   */
  bool init()
  {
  
    tfBuffer_ = new tf2_ros::Buffer; //optional parameter: ros::Duration(cache time) (default=10)
    tf_listener_ = new tf2_ros::TransformListener(*tfBuffer_);
    
    setupParams();
    setupPublishersSubscribers();

    
    
    this->enable();
    curr_index_ = -1;
    executing_ = false;
    
    return true;
  };

private:
  ros::NodeHandle nh_;
  std::string name_;
  
  tf2_ros::Buffer* tfBuffer_;
  tf2_ros::TransformListener* tf_listener_;
  
  ros::Subscriber enable_controller_subscriber_, disable_controller_subscriber_, odom_subscriber_, trajectory_subscriber_;
  ros::Publisher command_publisher_, trajectory_odom_publisher_, transformed_trajectory_publisher_;
  double k_turn_;
  double k_drive_;
  ros::Time start_time_;
  std::string odom_frame_, base_frame_;
  
  trajectory_generator::trajectory_points desired_trajectory_;
  size_t curr_index_;
  bool executing_;
  
  
  void setupPublishersSubscribers()
  {
    enable_controller_subscriber_ = nh_.subscribe("enable", 10, &TrajectoryController::enableCB, this);
    disable_controller_subscriber_ = nh_.subscribe("disable", 10, &TrajectoryController::disableCB, this);
    odom_subscriber_ = nh_.subscribe("/odom", 1, &TrajectoryController::OdomCB, this);
    trajectory_subscriber_ = nh_.subscribe("/desired_trajectory", 10, &TrajectoryController::TrajectoryCB, this);
    command_publisher_ = nh_.advertise< geometry_msgs::Twist >("/cmd_vel_mux/input/navi", 10);
    trajectory_odom_publisher_ = nh_.advertise< nav_msgs::Odometry >("/desired_odom", 10);
    transformed_trajectory_publisher_ = nh_.advertise< trajectory_generator::trajectory_points >("/transformed_trajectory", 10);
  }
  
  void setupParams()
  {
    nh_.param<std::string>("odom_frame", odom_frame_, "odom");
    nh_.param<std::string>("base_frame", base_frame_, "base_link");
    nh_.param<double>("k_drive", k_drive_, 1.0);
    nh_.param<double>("k_turn", k_turn_, 1.0);

  }

  

  /**
   * @brief ROS logging output for enabling the controller
   * @param msg incoming topic message
   */
  void enableCB(const std_msgs::EmptyConstPtr msg);

  /**
   * @brief ROS logging output for disabling the controller
   * @param msg incoming topic message
   */
  void disableCB(const std_msgs::EmptyConstPtr msg);

  /**
   * @brief Turns on/off a LED, when a bumper is pressed/released
   * @param msg incoming topic message
   */
  void OdomCB(const nav_msgs::OdometryPtr msg);
  
  void TrajectoryCB(const trajectory_generator::trajectory_points msg);
  

  nav_msgs::OdometryPtr getDesiredState(std_msgs::Header header);

  Eigen::Matrix2cd getComplexMatrix(double x, double y, double cosTh, double sinTh);

  geometry_msgs::Twist ControlLaw(nav_msgs::OdometryPtr current, nav_msgs::OdometryPtr desired);

};

void TrajectoryController::enableCB(const std_msgs::EmptyConstPtr msg)
{
  if (this->enable())
  {
    ROS_INFO_STREAM("Controller has been enabled. [" << name_ << "]");
    start_time_ = ros::Time::now();
  }
  else
  {
    ROS_INFO_STREAM("Controller was already enabled. [" << name_ <<"]");
  }
};

void TrajectoryController::disableCB(const std_msgs::EmptyConstPtr msg)
{
  if (this->disable())
  {
    ROS_INFO_STREAM("Controller has been disabled. [" << name_ <<"]");
    if(executing_)
    {
      executing_ = false;
      curr_index_ = -1;
      ROS_INFO_STREAM("Interrupted trajectory. [" << name_ <<"]");
    }
  }
  else
  {
    ROS_INFO_STREAM("Controller was already disabled. [" << name_ <<"]");
  }
};


void TrajectoryController::TrajectoryCB(const trajectory_generator::trajectory_points msg)
{
  ROS_INFO_STREAM("Trajectory received. [" << name_ <<"]");
  if (this->getState())
  {
    if(!executing_)
    {

      try
      {
        // Uses the time and frame provided by header of msg (tf2_ros::buffer_interface.h)
        desired_trajectory_ = msg;//tfBuffer_->transform(msg, odom_frame_);
      }
      catch (tf2::TransformException &ex) {
          ROS_ERROR("Unable to execute trajectory: %s",ex.what());
          return;
      }
      
      trajectory_odom_publisher_.publish(desired_trajectory_);
      ROS_INFO_STREAM("Preparing to execute. [" << name_ <<"]");
      executing_ = true;
      curr_index_ = 0;
    }
    else
    {
      ROS_INFO_STREAM("Already executing trajectory, new trajectory ignored. [" << name_ <<"]");
    }
  }
  else
  {
    ROS_INFO_STREAM("Controller disabled, will not execute trajectory. [" << name_ <<"]");
  }

}


void TrajectoryController::OdomCB(const nav_msgs::OdometryPtr msg)
{
  if (this->getState() && executing_) // check, if the controller is active
  {
  ROS_INFO_STREAM("Odom@ " << msg->header.stamp << "s: (" << msg->pose.pose.position.x << "," << msg->pose.pose.position.y << ") and " << msg->pose.pose.orientation.w <<"," << msg->pose.pose.orientation.z);
  
    nav_msgs::OdometryPtr desired = TrajectoryController::getDesiredState(msg->header);
    trajectory_odom_publisher_.publish(desired);
    geometry_msgs::Twist command = TrajectoryController::ControlLaw(msg, desired);
    command_publisher_.publish(command);
    ROS_INFO_STREAM("Command: " << command.linear.x <<"m/s, " << command.angular.z << "rad/s");
    
    if(curr_index_ == desired_trajectory_.points.size()-1)
    {
        executing_ = false;
        curr_index_ = -1;
    }
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

    ROS_INFO("\n%f + %fi, %f + %fi\n%f + %fi, %f + %fi", g.real()(0,0), g.imag()(0,0), g.real()(0,1), g.imag()(0,1), g.real()(1,0), g.imag()(1,0), g.real()(1,1), g.imag()(1,1));

  return g;
}



geometry_msgs::Twist TrajectoryController::ControlLaw(nav_msgs::OdometryPtr current, nav_msgs::OdometryPtr desired)
{
    geometry_msgs::Point position = current->pose.pose.position;
    geometry_msgs::Quaternion orientation = current->pose.pose.orientation;

    ROS_INFO_STREAM("Current:");
    Eigen::Matrix2cd g_curr = TrajectoryController::getComplexMatrix(position.x, position.y, orientation.w, orientation.z);

    position = desired->pose.pose.position;
    orientation = desired->pose.pose.orientation;

    ROS_INFO_STREAM("Desired:");
    Eigen::Matrix2cd g_des = TrajectoryController::getComplexMatrix(position.x, position.y, orientation.w, orientation.z);

    Eigen::Matrix2cd g_error = g_curr.inverse() * g_des;

    ROS_INFO("Error:\n%f + %fi, %f + %fi\n%f + %fi, %f + %fi", g_error.real()(0,0), g_error.imag()(0,0), g_error.real()(0,1), g_error.imag()(0,1), g_error.real()(1,0), g_error.imag()(1,0), g_error.real()(1,1), g_error.imag()(1,1));

    
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

    geometry_msgs::Twist command;
    command.linear = linear;
    command.angular = angular;

    ROS_INFO_STREAM("Linear Error: " << x_error << "m, Angular Error: " << theta_error << "rad");
    
    return command;
}

nav_msgs::OdometryPtr TrajectoryController::getDesiredState(std_msgs::Header header)
{
  
  ros::Duration elapsed_time = header.stamp - desired_trajectory_.header.stamp;
  double t = elapsed_time.toSec();

  //This 'should' update curr_index to refer to the last trajectory point before the desired time.
  for(; curr_index_ < desired_trajectory_.points.size()-1 && desired_trajectory_.points[curr_index_+1].time < elapsed_time; curr_index_++);

  ROS_INFO_STREAM("Index: " << curr_index_ << "; # points: " << desired_trajectory_.points.size()); 
 
  //This handles the case where we've reached the end of the trajectory time
  int post_index = std::min(curr_index_+1, desired_trajectory_.points.size()-1);
  
  ROS_INFO_STREAM("Preindex: " << curr_index_ << "; postindex: " << post_index);  
 
  trajectory_generator::trajectory_point pre_point = desired_trajectory_.points[curr_index_];
  trajectory_generator::trajectory_point post_point = desired_trajectory_.points[post_index];
  
  ros::Duration pre_time = elapsed_time - pre_point.time;
  ros::Duration period = post_point.time - pre_point.time;
  
  
  double pre_time_fraction = 1;
  if(curr_index_ < desired_trajectory_.points.size()-1)
  {
    pre_time_fraction = pre_time.toSec()/period.toSec();
  }

  double x = pre_point.x*(1-pre_time_fraction) + post_point.x*pre_time_fraction;
  double y = pre_point.y*(1-pre_time_fraction) + post_point.y*pre_time_fraction;;
  double theta = pre_point.theta*(1-pre_time_fraction) + post_point.theta*pre_time_fraction;
  double v = pre_point.v*(1-pre_time_fraction) + post_point.v*pre_time_fraction;
  double w = pre_point.w*(1-pre_time_fraction) + post_point.w*pre_time_fraction;

  //This was in the trajectory generator class.
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, theta);

  nav_msgs::OdometryPtr odom(new nav_msgs::Odometry);

  // Header
  odom->header.stamp = header.stamp;
  odom->header.frame_id = desired_trajectory_.header.frame_id;

  // Position
  odom->pose.pose.position.x = x;
  odom->pose.pose.position.y = y;
  odom->pose.pose.position.z = 0.0;
  odom->pose.pose.orientation = quat;

  // Velocity
  odom->twist.twist.linear.x = v;
  odom->twist.twist.linear.y = 0;
  odom->twist.twist.angular.z = w;

  ROS_INFO_STREAM("Desired@ " << t << "s: (" << x << "," << y << ") and " << quat.w <<"," << quat.z);

  return odom;
};

} // namespace kobuki
// %EndTag(FULLTEXT)%
#endif /* TRAJECTORY_CONTROLLER_HPP_ */
