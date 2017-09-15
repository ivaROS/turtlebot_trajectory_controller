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
//#include <trajectory_generator_ros_interface.h>
#include <pips_trajectory_msgs/trajectory_point.h>
#include <pips_trajectory_msgs/trajectory_points.h>

#include "rate_tracker.h"
#include <tf2_pips/tf2_trajectory.h>
#include <turtlebot_trajectory_controller/TurtlebotControllerConfig.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <yocs_controllers/default_controller.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>

#include <ros/callback_queue.h>

#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>

#include <memory>

typedef std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
typedef std::shared_ptr<tf2_ros::TransformListener> transform_listener_ptr;
typedef std::shared_ptr<ros::AsyncSpinner> spinner_ptr;
typedef tf2_ros::MessageFilter<pips_trajectory_msgs::trajectory_points> tf_filter;

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
  TrajectoryController(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~TrajectoryController()
  {
    if(odom_spinner_)
    {
      odom_spinner_->stop();
    }
  }

  /**
   * Set-up necessary publishers/subscribers
   * @return true, if successful
   */
  virtual bool init();

protected:
  ros::NodeHandle nh_;

private:
  std::string name_ = "RTController";
  ros::NodeHandle pnh_;
  void configCB(turtlebot_trajectory_controller::TurtlebotControllerConfig &config, uint32_t level);
  void setupPublishersSubscribers();
  void setupParams();
  
  typedef dynamic_reconfigure::Server<turtlebot_trajectory_controller::TurtlebotControllerConfig> ReconfigureServer;
  std::shared_ptr<ReconfigureServer> reconfigure_server_;

protected:
  bool send_command_=true;
  rate_tracker odom_rate;
  spinner_ptr odom_spinner_;
  ros::NodeHandle odom_nh_;
  ros::CallbackQueue odom_queue_;
  
  tf_buffer_ptr tfBuffer_;
  transform_listener_ptr tf_listener_;
  
  ros::Subscriber enable_controller_subscriber_, disable_controller_subscriber_, odom_subscriber_;
  ros::Publisher command_publisher_, trajectory_odom_publisher_, transformed_trajectory_publisher_;
  double k_turn_, k_drive_x_, k_drive_y_;
  ros::Time start_time_;
  std::string odom_frame_id_, base_frame_id_;
  int trajectory_queue_size_ = 1;
  bool use_odom_spinner_=false;
  
  pips_trajectory_msgs::trajectory_points desired_trajectory_;
  size_t curr_index_;
  bool executing_;
  
  nav_msgs::Odometry::ConstPtr curr_odom_;
  boost::mutex trajectory_mutex_;
  boost::mutex odom_mutex_;
  
  message_filters::Subscriber<pips_trajectory_msgs::trajectory_points> trajectory_subscriber_;
  std::shared_ptr<tf_filter> tf_filter_;
  

  /**
   * @brief ROS logging output for enabling the controller
   * @param msg incoming topic message
   */
  void enableCB(const std_msgs::Empty::ConstPtr& msg);

  /**
   * @brief ROS logging output for disabling the controller
   * @param msg incoming topic message
   */
  void disableCB(const std_msgs::Empty::ConstPtr& msg);
  
  void stop(bool force_stop=false);

  /**
   * @brief Turns on/off a LED, when a bumper is pressed/released
   * @param msg incoming topic message
   */
  virtual void OdomCB(const nav_msgs::Odometry::ConstPtr& msg);
  
  void TrajectoryCB(const pips_trajectory_msgs::trajectory_points::ConstPtr& msg);
  

  nav_msgs::OdometryPtr getDesiredState(const std_msgs::Header& header);

  Eigen::Matrix2cd getComplexMatrix(double x, double y, double cosTh, double sinTh);

  geometry_msgs::Twist::ConstPtr ControlLaw(const nav_msgs::Odometry::ConstPtr& current, const nav_msgs::Odometry::ConstPtr& desired);

};

} // namespace kobuki
// %EndTag(FULLTEXT)%
#endif /* TRAJECTORY_CONTROLLER_H_ */
