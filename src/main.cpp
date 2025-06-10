#include <ros/ros.h>

#include "ros_g29_logitech_controller/g29_force_feedback.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_g29_logitech_controller_node");
    G29ForceFeedback g29_ff;
    ros::spin();
    return(0);
}