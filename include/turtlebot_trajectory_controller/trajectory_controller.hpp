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

#ifndef BUMP_BLINK_CONTROLLER_HPP_
#define BUMP_BLINK_CONTROLLER_HPP_

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
    enable_controller_subscriber_ = nh_.subscribe("enable", 10, &TrajectoryController::enableCB, this);
    disable_controller_subscriber_ = nh_.subscribe("disable", 10, &TrajectoryController::disableCB, this);
    command_publisher_ = nh_.advertise< geometry_msgs::Twist >("cmd_vel_mux/input/navi", 10);
    this->disable();
    return true;
  };

private:
  ros::NodeHandle nh_;
  std::string name_;
  ros::Subscriber enable_controller_subscriber_, disable_controller_subscriber_;
  ros::Publisher command_publisher_;
  double k_turn_;
  double k_drive_;
  ros::Time start_time_;

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
  }
  else
  {
    ROS_INFO_STREAM("Controller was already disabled. [" << name_ <<"]");
  }
};

/*
void TrajectoryController::setTrajectory(const trajectory_msgs::JointTrajectoryConstPtr msg)
{

}
*/

Eigen::Matrix2cd TrajectoryController::getComplexMatrix(double x, double y, double cosTh, double sinTh)
{
  Eigen::Matrix2cd g(2,2);
  g.real()(0,0) = cosTh;
  g.real()(0,1) = x;
  g.real()(1,0) = 0;
  g.real()(1,1) = 1;

  g.imag()(0,0) = sinTh;
  g.imag()(0,1) = y;
  g.imag()(1,0) = 0;
  g.imag()(1,1) = 1;
  return g;
}



geometry_msgs::Twist TrajectoryController::ControlLaw(nav_msgs::OdometryPtr current, nav_msgs::OdometryPtr desired)
{
    geometry_msgs::Point position = current->pose.pose.position;
    geometry_msgs::Quaternion orientation = current->pose.pose.orientation;

    Eigen::Matrix2cd g_curr = TrajectoryController::getComplexMatrix(position.x, position.y, orientation.w, orientation.z);

    position = desired->pose.pose.position;
    orientation = desired->pose.pose.orientation;

    Eigen::Matrix2cd g_des = TrajectoryController::getComplexMatrix(position.x, position.y, orientation.w, orientation.z);

    Eigen::Matrix2cd g_error = g_curr.inverse() * g_des;

    
    double theta_error = std::arg(g_error(0,0));
    double x_error = g_error.real()(0,1);
    
    double v_ang_fb = theta_error * k_turn_;
    double v_lin_fb = x_error * k_drive_;

    double v_ang_ff = desired->twist.twist.angular.z;
    double v_lin_ff = desired->twist.twist.linear.x;

    double v_ang = v_ang_fb + v_ang_ff;
    double v_lin = v_lin_fb + v_lin_ff;

    geometry_msgs::Vector3 linear;
    linear.x = v_lin;

    geometry_msgs::Vector3 angular;
    angular.z = v_ang;

    geometry_msgs::Twist command;
    command.linear = linear;
    command.angular = angular;
    
    return command;
}

nav_msgs::OdometryPtr TrajectoryController::getDesiredState(std_msgs::Header header)
{
  
  ros::Duration elapsed_time = header.stamp - start_time_;
  double t = elapsed_time.toSec();


  double vel = .05;

  double x = t * vel;
  double y = 0;
  double theta = 0;

  

  geometry_msgs::Quaternion quat;
  quat.w = cos(theta);
  quat.z = sin(theta);

  nav_msgs::OdometryPtr odom(new nav_msgs::Odometry);

  // Header
  odom->header = header;

  // Position
  odom->pose.pose.position.x = x;
  odom->pose.pose.position.y = y;
  odom->pose.pose.position.z = 0.0;
  odom->pose.pose.orientation = quat;

  // Velocity
  odom->twist.twist.linear.x = 0;
  odom->twist.twist.linear.y = 0;
  odom->twist.twist.angular.z = 0;

  return odom;
}


void TrajectoryController::OdomCB(const nav_msgs::OdometryPtr msg)
{
  if (this->getState()) // check, if the controller is active
  {
    nav_msgs::OdometryPtr desired = TrajectoryController::getDesiredState(msg->header);
    geometry_msgs::Twist command = TrajectoryController::ControlLaw(msg, desired);
  }










/*




    // Preparing LED message
    kobuki_msgs::LedPtr led_msg_ptr;
    led_msg_ptr.reset(new kobuki_msgs::Led());

    if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
    {
      ROS_INFO_STREAM("Bumper pressed. Turning LED on. [" << name_ << "]");
      led_msg_ptr->value = kobuki_msgs::Led::GREEN;
      blink_publisher_.publish(led_msg_ptr);
    }
    else // kobuki_msgs::BumperEvent::RELEASED
    {
      ROS_INFO_STREAM("Bumper released. Turning LED off. [" << name_ << "]");
      led_msg_ptr->value = kobuki_msgs::Led::BLACK;
      blink_publisher_.publish(led_msg_ptr);
    }
  }
*/
};

} // namespace kobuki
// %EndTag(FULLTEXT)%
#endif /* BUMP_BLINK_CONTROLLER_HPP_ */
