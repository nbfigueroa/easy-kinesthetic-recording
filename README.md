# easy-kinesthetic-recording
Scripts and instructions to easily record data from kinesthetic demonstrations as rosbags and convert to matlab format.

---
### Kinesthetic Demonstration Recording:
To record/replay(bag) demonstrations you must install these packages:

| Dependencies  |
| ------------- |
| [record_ros](https://github.com/epfl-lasa/record_ros) |


### Instructions
##### Launch rviz to visualize Robot state :
```
$ roslaunch kuka_lwr_bringup lwr2_tabletop.launch robot_urdf_name:=kuka_grav_comp.xacro
```

##### Run KUKA-LWR-ROS Controller


##### Recorder node for all topics necessary
```
$ roslaunch easy_kinesthetic_recording record_demonstrations.launch
```

##### Start/Stop a Recording (Rosbag)
```
$ rosservice call /record/cmd "cmd: 'record/stop'"
```

### Replaying a recorded demonstration
##### Visualization
```
$ roslaunch kuka_lwr_bringup lwr_realtime.launch
```
##### Play bag
```
$ rosbag play *.bag
```

##### Extract topics to Mat file
Use  [my-matlab-rosbag](https://github.com/nbfigueroa/my_matlab_rosbag)

