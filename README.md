# easy-kinesthetic-recording
Scripts and instructions to easily record data from kinesthetic demonstrations as rosbags and convert to matlab format.

---
### Kinesthetic Demonstration Recording:
To record/replay(bag) demonstrations you must install these packages:

| Dependencies  |
| ------------- |
| [kuka-lwr-ros](https://github.com/epfl-lasa/kuka-lwr-ros.git) |
| [record_ros](https://github.com/epfl-lasa/record_ros) |

### Instructions
##### Run KUKA-LWR-ROS Controller
Assuming you have installed the [kuka-lwr-ros](https://github.com/epfl-lasa/kuka-lwr-ros.git) package, run the real-robot control interface and console in different terminals:
```
$ roslaunch lwr_simple_example real.launch
$ roslaunch lwr_fri lwr_fri_console.launch
```
Once the robot is in 'command' mode, you can put it in 'grav-comp' mode by:
```
$ ro....
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

