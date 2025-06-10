#include <ros/ros.h>

#include "ros_g29_logitech_controller/g29_force_feedback.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_g29_logitech_controller_node");
    ros::NodeHandle nh;

    using ros::param::param;
    const auto device_name = param<std::string>("~device_name", "/dev/input/event0");
    const auto loop_rate = param<double>("~loop_rate", 0.01);
    const auto max_torque = param<double>("~max_torque", 1.0);
    const auto min_torque = param<double>("~min_torque", 0.1);
    const auto brake_position = param<double>("~brake_position", 0.05);
    const auto brake_torque_rate = param<double>("~brake_torque_rate", 0.5);
    const auto auto_centering_max_torque = param<double>("~auto_centering_max_torque", 1.0);
    const auto auto_centering_max_position = param<double>("~auto_centering_max_position", 1.0);
    const auto eps = param<double>("~eps", 0.01);
    const auto auto_centering = param<bool>("~auto_centering", false);

    auto g29_ff = std::make_shared<G29ForceFeedback>(
        G29ForceFeedback::Configuration {
            .device_name = device_name,
            .loop_rate = loop_rate,
            .max_torque = max_torque,
            .min_torque = min_torque,
            .brake_position = brake_position,
            .brake_torque_rate = brake_torque_rate,
            .auto_centering_max_torque = auto_centering_max_torque,
            .auto_centering_max_position = auto_centering_max_position,
            .eps = eps,
            .auto_centering = auto_centering
        }
    );

    // When using lambda with capture as callback, the subscribe with template<M, C> must be used to force compiler to pick that overload
    ros::Subscriber feedback_sub =
        nh.subscribe<ros_g29_logitech_controller::ForceFeedback, const ros_g29_logitech_controller::ForceFeedback&>(
            "/g29/ff_target", 10,
            [g29_ff](const ros_g29_logitech_controller::ForceFeedback& msg) -> void
            {
                assert(g29_ff != nullptr && "g29_ff should not be nullptr");
                g29_ff->sendTargetFeedback(msg);
            }
        );

    ros::spin();

    return 0;
}