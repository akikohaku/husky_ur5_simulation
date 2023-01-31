# husky_ur5_simulation
A simulation environment for Husky robot with ur5
ubuntu 18 with ros melodic

## Installation

```
sudo apt-get install ros-melodic-husky-* ros-melodic-universal-robots
cd [workspace]/src
git clone https://github.com/akikohaku/husky_ur5_simulation.git
git clone https://github.com/lmark1/velodyne_simulator.git
git clone https://github.com/nilseuropa/realsense_ros_gazebo.git
catkin build
```

## How to use

### launch robot
```
roslaunch husky_ur5 husky_empty_world.launch
```

### launch planning class
```
roslaunch husky_ur5 moveit_planning_execution.launch sim:=true
```

### plan and execute arm
Add motionplanning in rviz and start planning.
