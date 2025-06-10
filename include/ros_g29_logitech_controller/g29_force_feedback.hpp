#pragma once

#include <ros/ros.h>

#include "ros_g29_logitech_controller/ForceFeedback.h"
#include <linux/input.h>



class G29ForceFeedback
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_target;
    ros::Timer timer;

    // device info
    int m_device_handle;
    int m_axis_code;
    int m_axis_min;
    int m_axis_max;

    // rosparam
    std::string m_device_name = "/dev/g29_steering_wheel_control";
    double m_loop_rate = 0.1;
    double m_max_torque = 1.0;
    double m_min_torque = 0.2;
    double m_brake_position = 0.2;
    double m_brake_torque_rate = 0.1;
    double m_auto_centering_max_torque = 0.3;
    double m_auto_centering_max_position = 0.2;
    double m_eps = 0.01;
    bool m_auto_centering = false;


    // variables
    ros_g29_logitech_controller::ForceFeedback m_target;
    bool m_is_target_updated = false;
    bool m_is_brake_range = false;
    struct ff_effect m_effect;
    double m_position;
    double m_torque;
    double m_attack_length;

public:
    G29ForceFeedback();
    ~G29ForceFeedback();

private:
    void targetCallback(const ros_g29_logitech_controller::ForceFeedback &in_target);
    void loop(const ros::TimerEvent&);
    int testBit(int bit, unsigned char *array);
    void initDevice();
    void calcRotateForce(double &torque, double &attack_length, const ros_g29_logitech_controller::ForceFeedback &target, const double &current_position);
    void calcCenteringForce(double &torque, const ros_g29_logitech_controller::ForceFeedback &target, const double &current_position);
    void uploadForce(const double &position, const double &force, const double &attack_length);
};
