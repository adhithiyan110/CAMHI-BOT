<h1 align ="center"> An Indoor Navigation Robot using SLAM </h1>


## Introduction
  The inclusion of automation in every fields has also led to it's place in navigation and here is a sample project of the same using components like Jetson Orin, Arduino, Motors and Encoders, Lidar etc. The main objective of this project is two phased. One involves the mapping of the surroundings with the use of lidars and SLAM. The second phase involves the Localising of the robot or in layman's terms ,finding the position of the robot relative to it's surroundings. Then it moves towards a destination given as input by finding the shortest path and avoiding potential obstacles along the way. The following are the steps and packages needed to replicate the project.
  

## Snaps of Final Product
<img src="https://github.com/user-attachments/assets/5a5df576-4ea9-4a3a-a857-3b1e8bf0aea4" width="300"/>
<img src="https://github.com/user-attachments/assets/1753dfdf-e155-4bd2-8798-309e448532fa" width="300"/>

## Working Demo of Final Product
https://github.com/user-attachments/assets/96d7b42f-637e-46dc-a6ae-6c6e2171f884


## General Installation 

```bash
sudo apt install gazebo
sudo apt install ros-foxy-gazebo-ros-pkgs
sudo apt install ros-foxy-twist-mux
```
```bash
sudo apt install ros-foxy-xacro
```

## URDF

```bash
sudo apt install ros-foxy-xacro ros-foxy-joint-state-publisher-gui
```
## LIDAR
```bash
sudo apt install ros-foxy-rplidar-ros
```
## ros_serial_demo
git clone in both devices
```bash
git clone https://github.com/joshnewans/serial_motor_demo.git
```
```bash
sudo apt install python3-serial
```
## SLAM TOOLBOX
```bash
sudo apt install ros-foxy-slam-toolbox
```
## NAV2
```bash
sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-turtlebot3*
```

- [Arduino code link](https://github.com/joshnewans/ros_arduino_bridge.git)


#### pin configuration


| Arduino pin | L298N pin     |
| :-------- | :------- | 
| D10 | IN2 | 
| D6   | IN1 |
| D9  | IN3 |
| D5  | IN4 |


| Arduino pin | Encoder pin    |
| :-------- | :------- | 
| D2 | left A | 
| D3   | left B |
| A4  | right A|
| A5  | right B |

The main commands to know are

- `e` - Motor responds with current encoder counts for each motor
- `r` - Reset encoder values
- `o <PWM1> <PWM2>` - Set the raw PWM speed of each motor (-255 to 255)  (example o 255 255)
- `m <Spd1> <Spd2>` - Set the closed-loop speed of each motor in *counts per loop* (Default loop rate is 30, so `(counts per sec)/30` (example m 115 115)
- `p <Kp> <Kd> <Ki> <Ko>` - Update the PID parameters

#### START COMMAND
```bash
miniterm -e /dev/ttyUSB0 57600
```

#### command in orin
```bash
ros2 run serial_motor_demo driver --ros-args -p encoder_cpr:=3440 -p loop_rate:=30 -p serial_port:=/dev/ttyUSB0 -p baud_rate:=57600
```

#### command in host
```bash
ros2 run serial_motor_demo gui
```

## ros2_control
```bash
sudo apt install ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-gazebo-ros2-control
```

```bash
ros2 control list_hardware_interfaces
```
```bash
ros2 control list_controllers
```
```bash
ros2 run controller_manager spawner.py diff_cont
```
```bash
ros2 run controller_manager spawner.py joint_broad
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```
## slam toolbox
```bash
cp /opt/ros/foxy/share/slam_toolbox/config/mapper_params_online_async.yaml omnisim_ws/src/omniwheel_description/config
```

## LAUNCH COMMANDS

```bash
ros2 launch rbot_description map.launch.py
```
```bash
ros2 launch rbot_description launch.robot.launch.py
```
```bash
ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB1 -p frame_id:=rp_lidar_1 -p angle_compensate:=true -p scan_mode:=Standard
ros2 launch rbot_description rplidar.launch.py
```
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```
```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/rbot_description/config/mapper_params_online_async.yaml use_time_time:=false
```
```bash
ros2 launch rbot_description localization_launch.py map:=./my_map.yaml use_sim_time:=false
```
```bash
ros2 launch rbot_description navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true
```
```bash
ros2 run twist_mux twist_mux --ros-args --params-file ./src/rbot_description/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped
```
```bash
ros2 launch realsense2_camera rs_launch.py
ros2 run rqt_image_view rqt_image_view
```

## LAUNCH COMMANDS NEW

```bash
ros2 launch rbot_description setup.launch.py
```
```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/rbot_description/config/mapper_params_online_async.yaml use_time_time:=false
```
```bash
ros2 launch rbot_description localization.launch.py map:=./my_map.yaml use_sim_time:=false
```
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true
```
