#ifndef RATE_TRACKER_H_
#define RATE_TRACKER_H_


#include <list>
#include <std_msgs/Header.h>

struct rate_tracker
{


private:
size_t max_size = 50;

std::list<ros::Time> times;
std::list<ros::Duration> delays;

size_t num_samples=0;


public:
  
  
rate_tracker(size_t window_size = 50) : max_size(window_size)
{
}

ros::Time now()
{
 return ros::Time::now();
}

void addTime()
{
  addTime(now()); 
}

void addTime(ros::Time time)
{
 
  if(times.size() == max_size)
    times.pop_front();
  
  num_samples = num_samples + 1;
  times.push_back(time);
}

void addDuration(ros::Time msgTime, ros::Time actualTime)
{
  if(delays.size() == max_size)
    delays.pop_front();

  ros::Duration delay = actualTime - msgTime;
  delays.push_back(delay);
}

void addTime(std_msgs::Header header)
{
  addTime(header.stamp);
  addDuration(header.stamp, now());
}

double getRate()
{
  if(times.size()<2)
    return 0;
  
  ros::Duration dt = times.back() - times.front();
  
  double rate = ((double)times.size()-1)/dt.toSec();
  return rate;
}

double getLastDelay()
{
  if(delays.size() == 0)
    return 0;
  else
    return delays.back().toSec();
}

double getAverageDelay()
{
  ros::Duration total_delay = ros::Duration(0);
  for (std::list<ros::Duration>::iterator it = delays.begin() ; it != delays.end(); ++it)
  {
    total_delay+=*it;
  }
  return total_delay.toSec()/((double)delays.size());
}

size_t getNumSamples()
{
  return delays.size();
}

};

#endif /* RATE_TRACKER_H_ */
