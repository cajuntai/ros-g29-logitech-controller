#pragma once

#include <memory>
#include <atomic>
#include <future>

#include <ros/ros.h>

#include "ros_g29_logitech_controller/ForceFeedback.h"
#include <linux/input.h>


class G29ForceFeedback
{
public:
    struct Configuration
    {
        std::string device_name;
        double loop_rate;
        double max_torque;
        double min_torque;
        double brake_position;
        double brake_torque_rate;
        double auto_centering_max_torque;
        double auto_centering_max_position;
        double eps;
        bool auto_centering = false;
    };

    G29ForceFeedback(Configuration config);
    ~G29ForceFeedback();
    auto sendTargetFeedback(const ros_g29_logitech_controller::ForceFeedback& feedback_msg) -> void;

private:
    ros::NodeHandle nh_;
    // ros::Subscriber sub_target;

    // ros::Timer timer;
    std::future<void> m_ff_loop_future;

    std::atomic_bool m_should_exit;

    // Device info
    int m_device_handle;
    int m_axis_code;
    int m_axis_min;
    int m_axis_max;
    // Behaviour configuration
    std::string m_device_name;
    double m_loop_rate;
    double m_max_torque;
    double m_min_torque;
    double m_brake_position;
    double m_brake_torque_rate;
    double m_auto_centering_max_torque;
    double m_auto_centering_max_position;
    double m_eps;
    bool m_auto_centering;

    ros_g29_logitech_controller::ForceFeedback m_target;
    bool m_is_target_updated = false;
    bool m_is_brake_range = false;
    struct ff_effect m_effect;
    double m_position;
    double m_torque;
    double m_attack_length;

    auto targetCallback(const ros_g29_logitech_controller::ForceFeedback& in_target) -> void;
    auto loop() -> void;
    auto testBit(int bit, unsigned char *array) -> int;
    auto initDevice() -> void;
    auto calcRotateForce(double& torque, double& attack_length, const ros_g29_logitech_controller::ForceFeedback& target, const double& current_position) -> void;
    auto calcCenteringForce(double& torque, const ros_g29_logitech_controller::ForceFeedback& target, const double& current_position) -> void;
    auto uploadForce(const double& position, const double& force, const double& attack_length) -> void;
};
