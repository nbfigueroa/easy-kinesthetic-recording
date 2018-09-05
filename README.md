# easy-kinesthetic-recording
Scripts and instructions to easily record data from kinesthetic demonstrations as rosbags and convert to matlab format.

---
### Kinesthetic Demonstration Recording:
To record/replay(bag) demonstrations you must install these packages:

| Dependencies  |
| ------------- |
| [kuka_interface_packages](https://github.com/nbfigueroa/kuka_interface_packages)    |
| [kuka-rviz-simulation](https://github.com/epfl-lasa/kuka-rviz-simulation)           |
| [record_ros](https://github.com/epfl-lasa/record_ros) |

### Instructions
##### Launch rviz to visualize Robot state :
```
$ roslaunch kuka_lwr_bringup lwr2_tabletop.launch robot_urdf_name:=kuka_grav_comp.xacro
```

##### Run KUKA Bridge to stream joint data:
```
$ rosrun kuka_fri_bridge run_lwr.sh
```
Run script up until ```monitor mode``` and set the KUKA to ```gravcomp``` in the control box. 

If you need specific joints locked or to demonstrate fast motions, set the bridge to ```Joint Impedance Control`` mode, see here (https://github.com/epfl-lasa/kuka_interface_packages) and run the following planning nodes:

###### Launch kuka-planning-interface:
Follow instructions in (https://github.com/nbfigueroa/kuka_planning_interface)

Set robot to:
```
$ rosservice call /control_cmd_interface/kuka_action_cmd 'grav_comp'
```
or
```
$ rosservice call /control_cmd_interface/kuka_action_cmd 'safe_grav_comp'
```

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

