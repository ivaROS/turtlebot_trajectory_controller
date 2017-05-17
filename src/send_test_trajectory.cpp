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
#include <memory>




namespace kobuki
{

/**
 * @ brief Sends a simple trajectory
 *
 * A simple program that sends a trajectory to the controller when a button is pressed.
 */
 
 //Generates a straight line trajectory with a given angle and speed
class angled_straight_traj_func : public traj_func{

    double dep_angle_;
    double v_;

public:
    angled_straight_traj_func( double dep_angle, double v ) : dep_angle_(dep_angle), v_(v) { }
    
    void dState ( const state_type &x , state_type &dxdt , const double  t  )
    {
        dxdt[near_identity::XD_IND] = v_*cos( dep_angle_);
        dxdt[near_identity::YD_IND] = v_*sin( dep_angle_);
    }
    
    
};

/* The rhs of x' = f(x) defined as a class */
class circle_traj_func : public traj_func{
    double vf_; //Forward vel
    double r_;  //radius of circle

public:
    circle_traj_func( double vf, double r) : vf_(vf), r_(r) { }
    

    void dState ( const state_type &x , state_type &dxdt , const double  t  )
    {
        
        dxdt[near_identity::YD_IND] = vf_*sin( - (vf_/r_) * t );
        dxdt[near_identity::XD_IND] = vf_*cos( - (vf_/r_) * t );
    }
    
    
};
//]

 
 
 
 
class TrajectoryTester 
{
public:
  TrajectoryTester(ros::NodeHandle& nh, std::string& name) : nh_(nh), name_(name){
  };
  ~TrajectoryTester(){
  };

  /**
   * Set-up necessary publishers/subscribers
   * @return true, if successful
   */
  bool init()
  {
    button_subscriber_ = nh_.subscribe("/mobile_base/events/button", 10, &TrajectoryTester::buttonCB, this);

    odom_subscriber_ = nh_.subscribe("/odom", 1, &TrajectoryTester::OdomCB, this);
    trajectory_publisher_ = nh_.advertise< pips_trajectory_msgs::trajectory_points >("/desired_trajectory", 10);
    path_publisher_ = nh_.advertise<nav_msgs::Path>("/desired_path",10);
    
    nav_msgs::OdometryPtr init_odom(new nav_msgs::Odometry);

    curOdom_ = init_odom;
    
    nh_.param<std::string>("/mobile_base/base_frame", base_frame_id_, "base_footprint");

    return true;
  };

private:
  ros::NodeHandle nh_;
  std::string name_;
  std::string base_frame_id_;
  ros::Subscriber button_subscriber_, odom_subscriber_;
  ros::Publisher trajectory_publisher_, path_publisher_;
  nav_msgs::OdometryPtr curOdom_;
  TrajectoryGeneratorBridge traj_gen_bridge;

  /**
   * @brief ROS logging output for enabling the controller
   * @param msg incoming topic message
   */
  void buttonCB(const kobuki_msgs::ButtonEventPtr& msg);

 
  /**
   * @brief Turns on/off a LED, when a bumper is pressed/released
   * @param msg incoming topic message
   */
  void OdomCB(const nav_msgs::OdometryPtr& msg);

  pips_trajectory_msgs::trajectory_points generate_circle_trajectory(const nav_msgs::OdometryPtr& msg);
  pips_trajectory_msgs::trajectory_points generate_straight_trajectory(const nav_msgs::OdometryPtr& msg);

};

void TrajectoryTester::buttonCB(const kobuki_msgs::ButtonEventPtr& msg)
{
  if (msg->button == kobuki_msgs::ButtonEvent::Button0 && msg->state == kobuki_msgs::ButtonEvent::RELEASED )
  {
    ROS_INFO_STREAM("Button pressed: sending trajectory");

    nav_msgs::OdometryPtr odom = nav_msgs::OdometryPtr(curOdom_);
    pips_trajectory_msgs::trajectory_points trajectory = TrajectoryTester::generate_circle_trajectory(odom);
    
    
    
    trajectory_publisher_.publish(trajectory);
  }
  else
  if (msg->button == kobuki_msgs::ButtonEvent::Button1 && msg->state == kobuki_msgs::ButtonEvent::RELEASED )
  {
    ROS_INFO_STREAM("Button pressed: sending trajectory");

    nav_msgs::OdometryPtr odom = nav_msgs::OdometryPtr(curOdom_);
    pips_trajectory_msgs::trajectory_points trajectory = TrajectoryTester::generate_circle_trajectory(odom);
    
    
    
    trajectory_publisher_.publish(trajectory);
  }
  else
  {
    ROS_INFO_STREAM("Button event");
  }
};


pips_trajectory_msgs::trajectory_points TrajectoryTester::generate_straight_trajectory(const nav_msgs::OdometryPtr& odom_msg)
{
    std::string offset_key, fw_vel_key, dep_angle_key;
    double fw_vel = .05;
    double offset = .5;
    double dep_angle = 0;

    if(ros::param::search("offset", offset_key))
    {
        ros::param::get(offset_key, offset); 
    }
    
    if(ros::param::search("fw_vel", fw_vel_key))
    {
        ros::param::get(fw_vel_key, fw_vel); 
    }
    
    if(ros::param::search("dep_angle", dep_angle_key))
    {
        ros::param::get(dep_angle_key, dep_angle); 
    }
    

    ni_trajectory_ptr traj = std::make_shared<ni_trajectory>();
    traj->header.frame_id = base_frame_id_;
    traj->header.stamp = odom_msg->header.stamp;
    
    traj->trajpntr = std::make_shared<angled_straight_traj_func>(dep_angle, fw_vel);
    traj->x0_ = traj_gen_bridge.initState();
    traj->x0_[near_identity::Y_IND] = offset;
    traj->params = std::make_shared<traj_params>(traj_gen_bridge.getDefaultParams());
      
      
    std::string key;
    double tf;
    
    if(ros::param::search("tf", key))
    {
        ros::param::get(key, tf); 
        traj->params->tf = tf;
    }
      
    traj_gen_bridge.generate_trajectory(traj);
    

    
    std::vector<ni_trajectory_ptr> trajectories;
    trajectories.push_back(traj);
    
    traj_gen_bridge.publishPaths(path_publisher_, trajectories);

    pips_trajectory_msgs::trajectory_points trajectory_msg = traj->toTrajectoryMsg ();
    
    return trajectory_msg;
}

pips_trajectory_msgs::trajectory_points TrajectoryTester::generate_circle_trajectory(const nav_msgs::OdometryPtr& odom_msg)
{
    std::string r_key, fw_vel_key;
    double fw_vel = .05;
    double r = .5;


    if(ros::param::search("r", r_key))
    {
        ros::param::get(r_key, r); 
    }
    
    if(ros::param::search("fw_vel", fw_vel_key))
    {
        ros::param::get(fw_vel_key, fw_vel); 
    }
    
    //circle_traj_func trajf(fw_vel,r);
    

    ni_trajectory_ptr traj = std::make_shared<ni_trajectory>();
    traj->header.frame_id = base_frame_id_;
    traj->header.stamp = odom_msg->header.stamp;
    
    traj->trajpntr = std::make_shared<circle_traj_func>(fw_vel,r);
    traj->x0_ = traj_gen_bridge.initState();
    traj->params = std::make_shared<traj_params>(traj_gen_bridge.getDefaultParams());
      
      
    std::string key;
    double tf;
    
    if(ros::param::search("tf", key))
    {
        ros::param::get(key, tf); 
        traj->params->tf = tf;
    }
      
    traj_gen_bridge.generate_trajectory(traj);
    

    
    std::vector<ni_trajectory_ptr> trajectories;
    trajectories.push_back(traj);
    
    traj_gen_bridge.publishPaths(path_publisher_, trajectories);


    pips_trajectory_msgs::trajectory_points trajectory_msg = traj->toTrajectoryMsg ();
    
    return trajectory_msg;
}



void TrajectoryTester::OdomCB(const nav_msgs::OdometryPtr& msg)
{

  curOdom_ = msg;
}

/*
void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

*/

} // namespace kobuki
// %EndTag(FULLTEXT)%


//http://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_trajectory_sender"); //, ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    std::string name = ros::this_node::getName();
    kobuki::TrajectoryTester tester(nh,name);
    
    //signal(SIGINT, mySigintHandler);

    if (tester.init())
    {
        ros::spin();
    }
    else
    {
        ROS_ERROR_STREAM("Couldn't initialise test_trajectory_sender!");
    }

    ROS_INFO_STREAM("Program exiting");
    return 0;

}

