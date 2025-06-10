# ros-g29-logitech-controller


# Features
* Two control modes

    * Control mode (`config/g29.yaml/auto_centering=false`)  
    Rotate wheel to the specified angle (`position`) with specified `torque` as specified with rostopic.

    * Auto centering mode (`config/g29.yaml/auto_centering=true`)  
    Automatically centering position, without publishing rostopic.


# Requirement
* Ubuntu 20.04
* ROS1 (Developed + tested on Noetic)
* Logitech G29 Driving Force Racing Wheel (Planning to test with g923)

To check whether your kernel supports force feedback, do as follows
```sh
cat $(ls /boot/config-* | head -1) | grep CONFIG_LOGIWHEELS_FF
# Output should be:
# CONFIG_LOGIWHEELS_FF=y
```  


# Installation

1. Download and build package
```sh
git clone https://github.com/cajuntai/ros-g29-logitech-controller.git
catkin build    # or catkin_make
```

2. (Optional) Set up a static port for G29 steering wheel (Make sure there is no existing static ports for the G29!)
```sh
# $G29_EVENT_FD_PATH: path to G29's event file descriptor (e.g., /dev/input/event12)
# $G29_STATIC_PORT_NAME: The static port name to give to G29
cd ros-g29-logitech-controller/scripts
sudo ./static_port_setup.sh $G29_EVENT_FD_PATH $G29_STATIC_PORT_NAME
```
    
# Usage
## Launch node
```sh
roslaunch ros_g29_logitech_controller g29_feedback.launch
```

## Control via rostopic
This message below moves the steering wheel to the original centre position at a torque of 0.1:
```sh
rostopic pub /ff_target ros_g29_force_feedback/msg/ForceFeedback "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, position: 0.0, torque: 0.1}"
```



# Config Parameters

**g29.yaml**
|parameter|default|description|
|:--|:--|:--|
|device_name|/dev/input/event19|device name, change the number|
|loop_rate|0.1|Loop of retrieving wheel position and uploading control to the wheel|
|max_torque|1.0|As for g29, 1.0 = 2.5Nm (min_torque < max_torque < 1.0)|
|min_torque|0.2|Less than 0.2 cannot rotate wheel|
|brake_torque|0.2|Braking torque to stop at the position (descrived below)|
|brake_position|0.1|Brake angle (`position`-0.1*max_angle)|
|auto_centering_max_torque|0.3|Max torque for auto centering|
|auto_centering_max_position|0.2|Max torque position while auto centering (`position`±0.2*max_angle)|
|eps|0.01|Wheel in the range (position-eps to position+eps) is considered as it has reached the `position`|
|auto_centering|false|Anto centering if true|

![ros_g29_ff](https://user-images.githubusercontent.com/38074802/167057448-1fa21956-ae91-4e51-bee4-1fcdc05cae51.png)
