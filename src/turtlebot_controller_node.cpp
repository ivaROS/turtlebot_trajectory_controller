#include <turtlebot_trajectory_controller/trajectory_controller.h>
#include <ros/ros.h>


int main(int argc, char **argv)
{
    std::string name= "turtlebot_controller";
    ros::init(argc, argv, name);
    ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    kobuki::TrajectoryController controller(nh, pnh);
    controller.init();


    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    //ros::shutdown();

	return 0;
}
