# Santec

This repository contains the ROS packages for simulation and web UI for control Santec - Disinfection robot

## Prerequisites
1. Ubuntu 20.04
2. Python 3.7
3. ROS Noetic

### Required ROS packages
- ros-noetic-controller-manager
- ros-noetic-joint-state-controller
- ros-noetic-navigation
- ros-noetic-robot-localization
- ros-noetic-ros-control
- ros-noetic-ros-controllers
- ros-noetic-teleop-twist-keyboard
- ros-noetic-tf2-geometry-msgs

## Development

### Setup
Santec packages use catkin_make for building code. So first of all you need to prepare catkin workspace:

```
$ mkdir -p santec_ws/src
$ cd santec_ws/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
```

Now you need to source the `setup.bash` created in catkin_ws/devel/ by previous build.

```
$ source ~/santec_ws/devel/setup.bash
$ echo "source ~/santec_ws/devel/setup.bash">> ~/.bashrc

```

Also add Gazebo models path for simulation

```
$ export GAZEBO_MODEL_PATH=$HOME/santec_ws/src/santec/santec_gazebo/models:$GAZEBO_MODEL_PATH
```

### Install dependencies

```
$ cd catkin_ws/src
$ git clone https://github.com/robotec-ua/web_video_server
$ git clone https://github.com/robotec-ua/follow_waypoints
$ git clone https://github.com/robotec-ua/robot_pose_publisher

```

### Build

Clone `santec` repository into santec_ws/src directory and execute `catkin_make` in the root of catkin workspace:

```
$ cd catkin_ws/src
$ git clone https://github.com/robotec-ua/robot_pose_publisher.git
$ cd ..
$ catkin_make
```

### Run simulation in Gazebosim

Run santec bot with GUI in Gazebosim with gmapping node

```
$ roslaunch santec_launch gmapping.launch
```

Run santec bot with GUI in Gazebosim with gmapping node
```
$ roslaunch santec_launch amcl.launch
```

Run santec bot with GUI in Gazebosim in `production mode`. In this case we can launch gmappin or amcl from GUI

```
$ roslaunch santec_launch basic_ui.launch
```

GUI will become available on your host Ubuntu OS at http://127.0.0.1:5000/ as well as from LAN.

## Santec GUI

### Robot control via teleoperation section 

In this section we have button for control robot moving.

`FOLLOW` button execute robot follow waypoints mode on points saved in pose.csv file.

`FORWARD`, `BACWARD`, `LEFT`, `RIGHT` buttons is teleoperation mode.


### Create map & waypoints section

In this section we have button for run gmapping and save new map and run amcl with follow waypoints for create new points and save it.

`BUILD MAP` and `SAVE MAP` buttons for run gmapping and save new map

`CHOOSE WP` and `SAVE WP` run amcl,follow wp nodes and save new wp 