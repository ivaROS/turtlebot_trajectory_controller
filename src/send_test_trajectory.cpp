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


/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <trajectory_generator_ros_interface.h>




namespace kobuki
{

/**
 * @ brief A simple bump-blink-controller
 *
 * A simple nodelet-based controller for Kobuki, which makes one of Kobuki's LEDs blink, when a bumper is pressed.
 */
 
 
//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
class sample_traj_func : public traj_func{

    double m_amp;
    double m_f;

public:
    sample_traj_func( double amp, double f ) : m_amp(amp), m_f(f) { }

    void init ( const state_type &x0 )
    {
        
    }
    
    void dState ( const state_type &x , state_type &dxdt , const double  t  )
    {
        dxdt[6] = 1;
        dxdt[7] = sin(t*2.0*3.14*m_f) * m_amp;
    }
    
    
};
//]

 
 
 
 
class TrajectoryTester 
{
public:
  TrajectoryTester(ros::NodeHandle& nh, std::string& name) : nh_(nh), name_(name){
      traj_gen_bridge = *(new TrajectoryGeneratorBridge);
  };
  ~TrajectoryTester(){};

  /**
   * Set-up necessary publishers/subscribers
   * @return true, if successful
   */
  bool init()
  {
    button_subscriber_ = nh_.subscribe("send", 10, &TrajectoryTester::buttonCB, this);

    odom_subscriber_ = nh_.subscribe("/odom", 1, &TrajectoryTester::OdomCB, this);
    trajectory_publisher_ = nh_.advertise< trajectory_generator::trajectory_points >("/desired_trajectory", 10);

    return true;
  };

private:
  ros::NodeHandle nh_;
  std::string name_;
  ros::Subscriber button_subscriber_, odom_subscriber_;
  ros::Publisher trajectory_publisher_;
  nav_msgs::OdometryPtr curOdom_;
  TrajectoryGeneratorBridge traj_gen_bridge;

  /**
   * @brief ROS logging output for enabling the controller
   * @param msg incoming topic message
   */
  void buttonCB(const kobuki_msgs::ButtonEventPtr msg);

 
  /**
   * @brief Turns on/off a LED, when a bumper is pressed/released
   * @param msg incoming topic message
   */
  void OdomCB(const nav_msgs::OdometryPtr msg);

  trajectory_generator::trajectory_points generate_trajectory(const nav_msgs::OdometryPtr msg);
  

};

void TrajectoryTester::buttonCB(const kobuki_msgs::ButtonEventPtr msg)
{
  if (msg->button == kobuki_msgs::ButtonEvent::Button0 && msg->state == kobuki_msgs::ButtonEvent::RELEASED )
  {
    ROS_INFO_STREAM("Button pressed: sending trajectory");

    nav_msgs::OdometryPtr odom = nav_msgs::OdometryPtr(curOdom_);
    trajectory_generator::trajectory_points trajectory = TrajectoryTester::generate_trajectory(odom);
    trajectory_publisher_.publish(trajectory);
  }
  else
  {
    ROS_INFO_STREAM("Button event");
  }
};




trajectory_generator::trajectory_points TrajectoryTester::generate_trajectory(const nav_msgs::OdometryPtr odom_msg)
{
  
    sample_traj_func traj(0.15,.1);
    
    traj_func* trajpntr = &traj;
    
  
    //start_time_ = ros::Time::now();
    std::vector<trajectory_generator::trajectory_point> points = traj_gen_bridge.generate_trajectory(odom_msg, trajpntr);
    
    trajectory_generator::trajectory_points trajectory_msg;
    trajectory_msg.points = points;
    trajectory_msg.header.stamp = ros::Time::now();
    
    return trajectory_msg;
}



void TrajectoryTester::OdomCB(const nav_msgs::OdometryPtr msg)
{

  curOdom_ = msg;
  }


} // namespace kobuki
// %EndTag(FULLTEXT)%



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_trajectory_sender");
    ros::NodeHandle nh;
    std::string name = ros::this_node::getName();
    kobuki::TrajectoryTester tester(nh,name);
    

    if (tester.init())
    {
        ros::spin();
    }
    else
    {
        ROS_ERROR_STREAM("Couldn't initialise KeyOpCore!");
    }

    ROS_INFO_STREAM("Program exiting");
    return 0;

}
