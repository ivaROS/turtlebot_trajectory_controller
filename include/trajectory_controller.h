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

#ifndef TRAJECTORY_CONTROLLER_H_
#define TRAJECTORY_CONTROLLER_H_

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <yocs_controllers/default_controller.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>
#include <trajectory_generator_ros_interface.h>
#include <tf/transform_datatypes.h>
#include <tf2_trajectory.h>
#include <tf2_ros/transform_listener.h>



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
  TrajectoryController(ros::NodeHandle& nh, std::string& name);;
  ~TrajectoryController(){};

  /**
   * Set-up necessary publishers/subscribers
   * @return true, if successful
   */
  virtual bool init();

protected:
  ros::NodeHandle nh_;
  std::string name_;
  
  tf2_ros::Buffer* tfBuffer_;
  tf2_ros::TransformListener* tf_listener_;
  
  ros::Subscriber enable_controller_subscriber_, disable_controller_subscriber_, odom_subscriber_, trajectory_subscriber_;
  ros::Publisher command_publisher_, trajectory_odom_publisher_, transformed_trajectory_publisher_;
  double k_turn_;
  double k_drive_;
  ros::Time start_time_;
  std::string odom_frame_id_, base_frame_id_;
  
  trajectory_generator::trajectory_points desired_trajectory_;
  size_t curr_index_;
  bool executing_;
  
  boost::mutex trajectory_mutex_;
  
  
  virtual void setupPublishersSubscribers();
  
  virtual void setupParams();

  

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
  
  void TrajectoryCB(const trajectory_generator::trajectory_points& msg);
  

  nav_msgs::OdometryPtr getDesiredState(std_msgs::Header header);

  Eigen::Matrix2cd getComplexMatrix(double x, double y, double cosTh, double sinTh);

  geometry_msgs::Twist ControlLaw(nav_msgs::OdometryPtr current, nav_msgs::OdometryPtr desired);

};

} // namespace kobuki
// %EndTag(FULLTEXT)%
#endif /* TRAJECTORY_CONTROLLER_H_ */
