#ifndef RATE_TRACKER_H_
#define RATE_TRACKER_H_


#include <list>
#include <std_msgs/Header.h>

/*Todo: use pass by reference when possible. Will need to overload since rvalues can't be passed by reference*/

struct rate_tracker
{


protected:
size_t max_size = 50;

std::list<ros::Time> times;
size_t num_total_samples=0;

inline
ros::Time now()
{
 return ros::Time::now();
}

public:


void addTime(ros::Time time)
{
 
  if(num_total_samples >= max_size)
    times.pop_front();
  
  num_total_samples = num_total_samples + 1;
  times.push_back(time);
}

inline
void addTime()
{
  addTime(now()); 
}

inline
double getNumRunningSamples()
{
   return (num_total_samples > max_size) ? max_size : num_total_samples;
}

void addTime(std_msgs::Header header)
{
  addTime(header.stamp);
}

double getRate()
{
  if(num_total_samples < 2)
    return 0;
  
  ros::Duration dt = times.back() - times.front();
  
  
  double rate = getNumRunningSamples()/dt.toSec();
  return rate;
}

size_t getNumSamples()
{
  return num_total_samples;
}

};

struct delay_tracker : rate_tracker
{
  private:
  std::list<ros::Duration> delays;

  public:
  double getLastDelay()
  {
    if(num_total_samples == 0)
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
  
    return total_delay.toSec()/getNumRunningSamples();
  }
  
  void addTime(const std_msgs::Header header)
  {
    rate_tracker::addTime(header);
    addDuration(header.stamp, now());
  }
  
  void addDuration(ros::Time msgTime, ros::Time actualTime)
  {
    if(num_total_samples >= max_size)
      delays.pop_front();

    ros::Duration delay = actualTime - msgTime;
    delays.push_back(delay);
  }
};

#endif /* RATE_TRACKER_H_ */
