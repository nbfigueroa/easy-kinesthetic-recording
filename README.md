# easy-kinesthetic-recording
Scripts and instructions to easily record data from kinesthetic demonstrations as rosbags and convert to matlab format using the Franka Panda Emika Robot.

---

## Installation

| Dependencies  |
| ------------- |
| [record_ros](https://github.com/epfl-lasa/record_ros) |
| [rosbag-to-mat](https://github.com/nbfigueroa/rosbag_to_mat) (If you want to export data to MATLAB)|
| [demo-voice-control](https://github.com/epfl-lasa/demo-voice-control.git) (Optional) |


To automagically install dependencies do the following steps:
* In your catkin src directory clone the repository
```
$ git clone -b latest-franka https://github.com/nbfigueroa/easy-kinesthetic-recording.git
```
* wstool gets all other git repository dependencies, after the following steps you should see extra catkin 
  packages in your src directory.
```
$  wstool init
$  wstool merge easy-kinesthetic-recording/dependencies.rosinstall 
$  wstool up 
```
* Query and installs all libraries and packages 
```
$ rosdep install --from-paths . --ignore-src --rosdistro noetic 
```

### Instructions
##### Run Franka-ROS-Kinesthetic Controller
Assuming you have installed the [kuka-lwr-ros](https://github.com/epfl-lasa/kuka-lwr-ros.git) package, run the real-robot control interface and console in different terminals:
```
$ roslaunch lwr_simple_example real.launch
$ roslaunch lwr_fri lwr_fri_console.launch
```
Once the robot is in 'command' mode, it is automatically in gravity compensation mode and you can move the robot around as you wish. You can also simply stay in 'command' mode, open the fri interface and put the robot in grav-comp mode via the teach pendant. What is the difference then?
- Recording demonstrations in 'command' mode, the frequency of ```/lwr/joint_states``` and ```/lwr/ee_pose``` is 500 hz (dt=0.002)
- Recording demonstrations in 'monitor' mode, the frequency of ```/lwr/joint_states``` and ```/lwr/ee_pose``` is 100 hz; (dt=0.01)


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

##### Control Topic Recorder with Voice Commands (Optional)
To control the ```rosservice call``` for the recorder node with voice commands, you should install and following the intructions in the voice control package https://github.com/epfl-lasa/demo-voice-control.git and run the launch file:
```
roslaunch demo_voice_control teach_voice_control.launch
```
### Examples
<p align="center">
<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/kuka-lwr-ros/img/Scenario1_demo.gif" width="350"><img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/kuka-lwr-ros/img/Scenario2_demo.gif" width="350">
</p>

If the variables ```<arg name="viz_traj"  	default="true"/>``` and ```<arg name="viz_obj" default="true"/>``` are set to ```true``` in the launch file, you should see the following in rviz. ```viz_obj``` only works if the robotiq gripper is used.
<p align="center">
<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/kuka-lwr-ros/img/Scenario1_rviz.gif" width="340"><img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/kuka-lwr-ros/img/Scenario2_rviz.gif" width="350">
</p>

### Replaying a recorded demonstration
You can replay the recorded demonstrations by running the following commands:
##### Visualization
```
$ roslaunch easy_kinesthetic_recording replay_bag_demonstrations.launch
```
##### Play bag
```
$ rosbag play *.bag
```

### Extracting Data to Matlab
To export the data recorded in the rosbags to matlab you can use the package [my-matlab-rosbag](https://github.com/nbfigueroa/my_matlab_rosbag) package. In the folder ``` my_matlab_rosbag/tasks/``` you will find a script that processes the rosbag topics and converts them to mat file. The ```corl_demos.m``` script will generate the following plots:
<p align="center">
<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/kuka-lwr-ros/img/Scenario1.png" width="400"><img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/kuka-lwr-ros/img/Scenario2.png" width="400">
</>

In these case, the recorded trajectories are labeled using the robotiq gripper state, hence segmentation of the recordings is straightforward; i.e. 
- when the gripper is closed, this indicates the non-linear motion that must be learned
- when the gripper is open, this indicates a "picking" or "back" motion that can also be learned

In the folder ``` my_matlab_rosbag/trajectory-processing/``` the script named: ```extract_trajectories.m``` will generate data structures containing the trajectories corresponding to each primitive. It will compute the velocities of the end effector positions using the Savitsky-Golay filter. Hence the script will generate the following data:

- For Scenario 1
<p align="center">
<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/kuka-lwr-ros/img/Scenario1_prim1_processed.png" width="850">
  <img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/kuka-lwr-ros/img/Scenario1_prim0_processed.png" width="850">
</p>

- For Scenario 2
<p align="center">
<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/kuka-lwr-ros/img/Scenario2_prim1_processed.png" width="850">
  <img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/kuka-lwr-ros/img/Scenario2_prim0_processed.png" width="850">
</p>

For more complex scenarios where the gripper state is not a sufficient indication of a phase/action change, you should segment the trajectories. 

<!-- possibly with my segmentation algorithm: https://github.com/nbfigueroa/ICSC-HMM or Lucia's Constraint-based approach https://ieeexplore.ieee.org/document/7339616/ -->


**Contact**: [Nadia Figueroa](https://nbfigueroa.github.io/) (nadiafig AT seas dot upenn dot edu)







