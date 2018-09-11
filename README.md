# easy-kinesthetic-recording
Scripts and instructions to easily record data from kinesthetic demonstrations as rosbags and convert to matlab format.

---
### Kinesthetic Demonstration Recording:
To record/replay(bag) demonstrations you must install these packages:

| Dependencies  |
| ------------- |
| [kuka-lwr-ros](https://github.com/epfl-lasa/kuka-lwr-ros.git) |
| [record_ros](https://github.com/epfl-lasa/record_ros) |
| [demo-voice-control](https://github.com/epfl-lasa/demo-voice-control.git) (Optional) |
| [robotiq](https://github.com/epfl-lasa/lasa-wiki/wiki/Robotiq-gripper) (Optional) |

### Instructions
##### Run KUKA-LWR-ROS Controller
Assuming you have installed the [kuka-lwr-ros](https://github.com/epfl-lasa/kuka-lwr-ros.git) package, run the real-robot control interface and console in different terminals:
```
$ roslaunch lwr_simple_example real.launch
$ roslaunch lwr_fri lwr_fri_console.launch
```
Once the robot is in 'command' mode, it is automatically in gravity compensation mode and you can move the robot around as you wish.

##### Run Topic Recorder
In the launch file ```launch/record_demonstrations.launch``` you can define the topics that you wish to record in the following argument.
```
<arg name="topic" default="/lwr/ee_pose /lwr/ee_vel /lwr/joint_states  /tf"/>
```
You must also define the path to the directory where all bags will be recorded and the bag prefix-:
```
<arg name="path_save"   default="/home/kinesthetic_recordings/bags"/>
<arg name="file_name"   default="demo_x"/>
```
Once you've done this, you can run the following launch file:
```
$ roslaunch easy_kinesthetic_recording record_demonstrations.launch
```
and when you are ready you can start/Stop a Recording (Rosbag) by typing the following in a terminal:
```
$ rosservice call /record/cmd "cmd: 'record/stop'"
```
#### Examples
<p align="center">
<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/kuka-lwr-ros/img/Scenario1_demo.gif" width="350"><img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/kuka-lwr-ros/img/Scenario2_demo.gif" width="350">
</p>

If the varables ```<arg name="viz_traj"  	default="true"/>``` and ```<arg name="viz_obj" default="true"/>``` are set to ```true``` in the launch file, you should see the following in rviz. ```viz_obj``` only works if the robotiq gripper is used.
<p align="center">
<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/kuka-lwr-ros/img/Scenario1_rviz.gif" width="340"><img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/kuka-lwr-ros/img/Scenario2_rviz.gif" width="350">
</p>

##### Control Topic Recorder with Voice Commands (Optional)
To control the ```rosservice call``` for the recorder node with voice commands, you should install and following the intructions in the voice control package https://github.com/epfl-lasa/demo-voice-control.git and run the launch file:
```
roslaunch demo_voice_control teach_voice_control.launch
```

### Replaying a recorded demonstration
##### Visualization
```
$ roslaunch easy_kinesthetic_recording replay_bag_demonstrations.launch
```
##### Play bag
```
$ rosbag play *.bag
```

##### Extract topics to Mat file
Use  [my-matlab-rosbag](https://github.com/nbfigueroa/my_matlab_rosbag)

