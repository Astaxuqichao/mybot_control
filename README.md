# mybot_control

There are ros_packages of mybot_control, include the following:
1. mecanumbot controller
2. diffbot hardware driver
3. mecanumbot hardware driver
4. diffbot and mecanumbot description
5. launch file
6. odom test scripts

**Those packages depend on ros-control and ros-controllers.**

## Schema

```
.
├── README.md
├── mybot_control      #meta package
├── mybot_controllers  #custom controllers
├── mybot_description  #robot description
├── mybot_hw           #robot hardware
├── mybot_launch       #launch file
└── mybot_test         #test scripts
```

## Input & Output 

_I use rosserial to connect the hardware and embedded, so i just need to manager input and output topic_  
_Input topic is the encoder state_  
_Output topic is the motor speed(command)_  

You can edit it to your own rules in `mybot_hw`

input(topic) | datetype | unit
---- | ---- | ----
wheel_state_encoder | std_msgs/Float32MultiArray | rad

output(topic) | datetype | unit
---- | ---- | ----
wheel_cmd_velocity | std_msgs/Float32MultiArray | rad/s


## USAGE
- make
```
#cd your_ws/src
#git clone this repository to your own workspace
catkin_make
```

- run diffbot with gazebo(use gazebo_ros_control)
```
source devel/setup.bash
roslaunch mybot_launch diffbot_gazebo_sim.launch
```

- run diffbot with real hardware
```
source devel/setup.bash
roslaunch mybot_launch diffbot_real.launch
```
note: diffbot driver uses the diffbot_controller in ros-controllers.

- run mecanumbot with real hardware
```
source devel/setup.bash
roslaunch mybot_launch diffbot_real.launch
```
