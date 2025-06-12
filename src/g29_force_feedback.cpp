#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <thread>
#include <chrono>
#include <future>

#include "ros_g29_logitech_controller/ForceFeedback.h"

#include "ros_g29_logitech_controller/g29_force_feedback.hpp"

constexpr double MAX_POSITION_POS = 1.0;
constexpr double MAX_POSITION_NEG = -1.0;


G29ForceFeedback::G29ForceFeedback(const Configuration config)
:   m_axis_code(ABS_X),
    m_should_publish_ff(true)
{
    m_device_name                 = config.device_name;
    m_loop_rate                   = config.loop_rate;
    m_max_torque                  = config.max_torque;
    m_min_torque                  = config.min_torque;
    m_brake_position              = config.brake_position;
    m_brake_torque_rate           = config.brake_torque_rate;
    m_auto_centering_max_torque   = config.auto_centering_max_torque;
    m_auto_centering_max_position = config.auto_centering_max_position;
    m_eps                         = config.eps;
    m_auto_centering              = config.auto_centering;

    initDevice();

    std::this_thread::sleep_for(std::chrono::seconds(1));   // Grace time after opening device
    m_ff_loop_future = std::async(std::launch::async, &G29ForceFeedback::loop, this);
}


G29ForceFeedback::~G29ForceFeedback()
{
    m_should_exit.store(true);
    if (m_ff_loop_future.valid())
    {
        try
        {
            std::cout << "Stopping control thread...";
            m_ff_loop_future.get();
            std::cout << "Done!" << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "[ERROR] Caught exception when stopping control thread: " << e.what() << std::endl;
        }
    }

    m_effect.type = FF_CONSTANT;
    m_effect.id = -1;
    m_effect.u.constant.level = 0;
    m_effect.direction = 0;

    // upload m_effect
    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0)
    {
        std::cout << "Failed to upload m_effect" << std::endl;
    }
}


// start force feedback loop
auto G29ForceFeedback::start() -> void
{
    if (!m_should_publish_ff.load())
    {
        m_should_publish_ff.store(true);
        std::cout << "Force feedback loop started." << std::endl;
    }
    else
    {
        std::cout << "Force feedback loop is already running." << std::endl;
    }
}

// stop force feedback loop
auto G29ForceFeedback::stop() -> void
{
    if (m_should_publish_ff.load())
    {
        m_should_publish_ff.store(false);
        std::cout << "Force feedback loop stopped." << std::endl;
    }
    else
    {
        std::cout << "Force feedback loop is already stopped." << std::endl;
    }
}

// update input event with timer callback
auto G29ForceFeedback::loop() -> void
{
    struct input_event event;
    // double last_position = m_position;

    while (!m_should_exit.load())
    {
        if (!m_should_publish_ff.load())
        {
            std::cout << "Force feedback loop is paused. Discarding FF message." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(m_loop_rate)));
            continue;
        }

        // get current state
        while (read(m_device_handle, &event, sizeof(event)) == sizeof(event))
        {
            if (event.type == EV_ABS && event.code == m_axis_code)
            {
                m_position = (event.value - (m_axis_max + m_axis_min) * 0.5) * 2 / (m_axis_max - m_axis_min);
            }
        }

        if (m_is_brake_range || m_auto_centering)
        {
            calcCenteringForce(m_torque, m_target, m_position);
            m_attack_length = 0.0;
        }
        else
        {
            calcRotateForce(m_torque, m_attack_length, m_target, m_position);
            m_is_target_updated = false;
        }

        uploadForce(m_target.position, m_torque, m_attack_length);
        std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(m_loop_rate)));
    }
}


auto G29ForceFeedback::calcRotateForce(double& torque,
                                       double& attack_length,
                                       const ros_g29_logitech_controller::ForceFeedback& target,
                                       const double& current_position) -> void
{
    double diff = target.position - current_position;
    double direction = (diff > 0.0) ? MAX_POSITION_POS : MAX_POSITION_NEG;

    if (fabs(diff) < m_eps)
    {
        torque = 0.0;
        attack_length = 0.0;
    }
    else if (fabs(diff) < m_brake_position)
    {
        m_is_brake_range = true;
        torque = target.torque * m_brake_torque_rate * -direction;
        attack_length = m_loop_rate;
    }
    else
    {
        torque = target.torque * direction;
        attack_length = m_loop_rate;
    }
}


auto G29ForceFeedback::calcCenteringForce(double &torque,
                                          const ros_g29_logitech_controller::ForceFeedback &target,
                                          const double &current_position) -> void
{
    double diff = target.position - current_position;
    double direction = (diff > 0.0) ? MAX_POSITION_POS : MAX_POSITION_NEG;

    if (fabs(diff) < m_eps)
    {
        torque = 0.0;
    }
    else
    {
        double torque_range = m_auto_centering_max_torque - m_min_torque;
        double power = (fabs(diff) - m_eps) / (m_auto_centering_max_position - m_eps);
        double buf_torque = power * torque_range + m_min_torque;
        torque = std::min(buf_torque, m_auto_centering_max_torque) * direction;
    }
}


// update input event with writing information to the event file
auto G29ForceFeedback::uploadForce(const double&, const double& torque, const double& attack_length) -> void
{
    // set effect
    m_effect.u.constant.level = 0x7fff * std::min(torque, m_max_torque);
    m_effect.direction = 0xC000;
    m_effect.u.constant.envelope.attack_level = 0; /* 0x7fff * force / 2 */
    m_effect.u.constant.envelope.attack_length = attack_length;
    m_effect.u.constant.envelope.fade_level = 0;
    m_effect.u.constant.envelope.fade_length = attack_length;

    // upload effect
    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0)
    {
        std::cout << "failed to upload effect" << std::endl;
    }
}


auto G29ForceFeedback::sendTargetFeedback(const ros_g29_logitech_controller::ForceFeedback& feedback_msg) -> void
{
    if (m_target.position == feedback_msg.position && m_target.torque == fabs(feedback_msg.torque))
    {
        m_is_target_updated = false;
    }
    else
    {
        m_target = feedback_msg;
        m_target.torque = fabs(m_target.torque);
        m_is_target_updated = true;
        m_is_brake_range = false;
    }
}



// initialize force feedback device
auto G29ForceFeedback::initDevice() -> void
{
    // setup device
    // unsigned char key_bits[1+KEY_MAX/8/sizeof(unsigned char)];
    unsigned char abs_bits[1+ABS_MAX/8/sizeof(unsigned char)];
    unsigned char ff_bits[1+FF_MAX/8/sizeof(unsigned char)];
    struct input_event event;
    struct input_absinfo abs_info;

    m_device_handle = open(m_device_name.c_str(), O_RDWR|O_NONBLOCK);
    if (m_device_handle < 0)
    {
        std::cout << "ERROR: cannot open device : "<< m_device_name << std::endl;
        throw std::runtime_error("Failed to open device!");
    }
    else
    {
        std::cout << "device opened" << std::endl;
    }

    // which axes has the device?
    memset(abs_bits, 0, sizeof(abs_bits));
    if (ioctl(m_device_handle, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) < 0)
    {
        std::cout << "ERROR: cannot get abs bits" << std::endl;
        throw std::runtime_error("Failed to open device!");
    }

    // get some information about force feedback
    memset(ff_bits, 0, sizeof(ff_bits));
    if (ioctl(m_device_handle, EVIOCGBIT(EV_FF, sizeof(ff_bits)), ff_bits) < 0)
    {
        std::cout << "ERROR: cannot get ff bits" << std::endl;
        throw std::runtime_error("Failed to open device!");
    }

    // get axis value range
    if (ioctl(m_device_handle, EVIOCGABS(m_axis_code), &abs_info) < 0)
    {
        std::cout << "ERROR: cannot get axis range" << std::endl;
        throw std::runtime_error("Failed to open device!");
    }

    // Commented due to constant faulty error
    m_axis_max = abs_info.maximum;
    m_axis_min = abs_info.minimum;
    std::cout << "axis range: [" << m_axis_min << ", " << m_axis_max << "]" << std::endl;
    if (m_axis_min >= m_axis_max)
    {
        std::cout << "ERROR: axis range has bad value" << std::endl;
        throw std::runtime_error("Failed to open device!");
    }

    // check force feedback is supported?
    if(!testBit(FF_CONSTANT, ff_bits))
    {
        std::cout << "ERROR: force feedback is not supported" << std::endl;
        throw std::runtime_error("Failed to open device!");
    }
    else
    {
        std::cout << "force feedback supported" << std::endl;
    }

    // auto centering off
    memset(&event, 0, sizeof(event));
    event.type = EV_FF;
    event.code = FF_AUTOCENTER;
    event.value = 0;
    if (write(m_device_handle, &event, sizeof(event)) != sizeof(event))
    {
        std::cout << "failed to disable auto centering" << std::endl;
        throw std::runtime_error("Failed to open device!");
    }

    // init effect and get effect id
    memset(&m_effect, 0, sizeof(m_effect));
    m_effect.type = FF_CONSTANT;
    m_effect.id = -1; // initial value
    m_effect.trigger.button = 0;
    m_effect.trigger.interval = 0;
    m_effect.replay.length = 0xffff;  // longest value
    m_effect.replay.delay = 0; // delay from write(...)
    m_effect.u.constant.level = 0;
    m_effect.direction = 0xC000;
    m_effect.u.constant.envelope.attack_length = 0;
    m_effect.u.constant.envelope.attack_level = 0;
    m_effect.u.constant.envelope.fade_length = 0;
    m_effect.u.constant.envelope.fade_level = 0;

    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0)
    {
        std::cout << "failed to upload m_effect" << std::endl;
        throw std::runtime_error("Failed to open device!");
    }

    // start m_effect
    memset(&event, 0, sizeof(event));
    event.type = EV_FF;
    event.code = m_effect.id;
    event.value = 1;
    if (write(m_device_handle, &event, sizeof(event)) != sizeof(event))
    {
        std::cout << "failed to start event" << std::endl;
        throw std::runtime_error("Failed to open device!");
    }
}


// util for initDevice()
auto G29ForceFeedback::testBit(int bit, unsigned char *array) -> int
{
    return ((array[bit / (sizeof(unsigned char) * 8)] >> (bit % (sizeof(unsigned char) * 8))) & 1);
}
