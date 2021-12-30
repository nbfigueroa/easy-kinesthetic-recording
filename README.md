# easy-kinesthetic-recording
Scripts and instructions to easily record data from kinesthetic demonstrations as rosbags and convert to matlab (or python - experimental) using the Franka Panda Emika Robot.

---

## Installation

| Dependencies  |
| ------------- |
| [franka_interactive_controllers](https://github.com/nbfigueroa/franka_interactive_controllers) |
| [record_ros](https://github.com/nbfigueroa/record_ros) |
| [rosbag_to_mat](https://github.com/nbfigueroa/rosbag_to_mat) (Working - If you want to export data to MATLAB)|
| [bagpy](https://jmscslgroup.github.io/bagpy/) (Experimental - If you want to export data to Python)|

<!-- | [demo-voice-control](https://github.com/epfl-lasa/demo-voice-control.git) (Optional) | -->


To automagically install dependencies do the following steps:
* In your catkin src directory clone the repository
```bash
$ git clone -b latest-franka https://github.com/nbfigueroa/easy-kinesthetic-recording.git
```
* wstool gets all other git repository dependencies, after the following steps you should see extra catkin 
  packages in your src directory.
```bash
$  wstool init
$  wstool merge easy-kinesthetic-recording/dependencies.rosinstall 
$  wstool up 
```
* Query and installs all libraries and packages 
```bash
$ rosdep install --from-paths . --ignore-src --rosdistro noetic 
```

---
## Step 1: Recording Kinesthetic Demonstrations as ROSBags

### Bringup Kinesthetic Teaching Pipeline
#### Run Franka-ROS-Kinesthetic Controller
Here we assume you have installed the [franka_interactive_controllers](https://github.com/nbfigueroa/franka_interactive_controllers) package and know how to use it. 

In two terminals you should launch the following:
```bash
roslaunch franka_interactive_controllers franka_interactive_bringup.launch
```
```bash
roslaunch franka_interactive_controllers joint_gravity_compensation_controller.launch
```

##### Run Topic Recorder
In the launch file ```launch/franka_record_demonstrations.launch``` you can define the topics that you wish to record in the following argument.
```xml
	<arg name="topic"  	    
		default="/tf 
		/franka_state_controller/joint_states 
		/franka_state_controller/F_ext 
		/franka_state_controller/O_T_EE 
		/franka_state_controller/O_T_FL 
		/franka_gripper/joint_states"/>	
```
You must also define the path to the directory where all bags will be recorded and the bag prefix-:
```xml
<arg name="path_save"      default="/home/panda2/rosbag_recordings/cooking/"/>
<arg name="file_name"  	   default="demo"/>
```
Once you've done this, you can run the following launch file:
```bash
roslaunch easy_kinesthetic_recording franka_record_demonstrations.launch
```

**Alternatively**, you can launch the following launch file from [franka_interactive_controllers](https://github.com/nbfigueroa/franka_interactive_controllers) that will bringup both the joint gravity compensation controllers and the topic recording launch file: 
```bash
roslaunch franka_interactive_controllers franka_kinesthetic_teaching.launch
```

You should now see the following displayed in your screen (without the trajectories):

<p align="center">
<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/latest-franka/img/franka_kinesthetic_teaching.png" width="800x">
</p>


**NOTE: If you run this script and the robot moves by itself, that means that your external_tool_compensation forces are incorrect. See external_tool_compensation instructions to correct it.**

### Record Demonstrations as ROSbags
To record/stop a rosbag recording you can either do it by: 
- Pressing the buttons on the GUI as shown in the image above
- Type the following in a terminal
```bash
 rosservice call /record/cmd "cmd: 'record/stop'"
 ```
 
<!-- - Control Topic Recorder with Voice Commands (Optional - TBD for Franka)
To control the ```rosservice call``` for the recorder node with voice commands, you should install and following the intructions in the voice control package https://github.com/epfl-lasa/demo-voice-control.git and run the launch file:
```bash
roslaunch demo_voice_control teach_voice_control.launch
``` -->

### Replaying a recorded demonstration
You can replay the recorded demonstrations by running the following commands:

##### Visualization
```bash
roslaunch easy_kinesthetic_recording franka_replay_bag_demonstrations.launch
```
##### Play bag
```bash
$ rosbag play *.bag
```
If the following variables are set to ``true``: 
- ``<arg name="viz_traj"  default="true"/>`` 
- ``<arg name="viz_obj" default="true"/>`` 

You can see the trajectories being replayed with the franka, the gripper will not be shown and you might see some erros in rviz, but that's fine:
The green block represent the state of the gripper:
- green: an object is grasped
- gray: no object is grasped

See examples below.

<!-- <p align="center">
	<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/latest-franka/img/scooping_rosbag_replay.gif" width="500x">
</p>
 -->

### Examples

This code together with [franka_interactive_controllers](https://github.com/nbfigueroa/franka_interactive_controllers) has been used for two household tasks:
- **cooking preparation task**: scooping and mixing ingredients from bowls

<p align="center">
	<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/latest-franka/img/scooping_task_reduced.gif" width="375x">
<!-- 	<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/latest-franka/img/scooping_recording.gif" width="400x">  -->
	<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/latest-franka/img/scooping_rosbag_replay.gif" width="400x">
</p>
<p align="center">
	Left: Video of kinesthetic demonstration, Right: Visualization of recorded trajectories by replaying recorded rosbag
</p>

- **table setting task**: grasping plates/cutlery from dish rack and placing it on a table.
*To Fill..*

---

## Step 2: Extracting ROSBag Data for Motion Policy Learning

### Extracting ROSBag Data to MATLAB (Working)
To export the data recorded in the rosbags to MATLAB you can use the [rosbag_to_mat](https://github.com/nbfigueroa/rosbag_to_mat) package. Follow the instructions in the README file to extract data for the following tasks:

#### cooking preparation task
Raw trajectories from demonstrations (colors indicate continuous demonstration):
<p align="center">
  <img src="https://github.com/nbfigueroa/rosbag_to_mat/blob/main/figs/franka-cooking-multistep.png" width="500x"> 
</p>

Segmented and processed trajectories from demonstrations (colors indicate trajectory clusters):


- **table setting task**:

### Extracting ROSBag Data to Python
This functionality hasn't been tested yet but I suggest to try out the [bagpy](https://jmscslgroup.github.io/bagpy/): a python package provides specialized class bagreader to read and decode ROS messages from bagfiles in just a few lines of code. 

---
## Contact
[Nadia Figueroa](https://nbfigueroa.github.io/) (nadiafig AT seas dot upenn dot edu)







