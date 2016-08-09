# easy-kinesthetic-recording
Scripts and instructions to easily record data from kinesthetic demonstrations as rosbags and convert to matlab format.

---
###Kinesthetic Demonstratio Setup:

#### To use the allegro hand to hold an object or tool
Install: [allegro-lib](https://github.com/nbfigueroa/allegro-lib)

##### Controlling the allegro hand:
To run the control interface:
```
$ rosrun allegrolib allegrolib
```

If you just want to simply open/close the hand, run theis node and follow the instructions on the command line:

```
$ rosrun allegrolib test_planner
```

#### To use the barret hand to hold an object or tool
Install: [http://wiki.ros.org/barrett_hand](http://wiki.ros.org/barrett_hand)
Easiest way to open/close move fingers is to use the GUI:
```
rosrun rqt_bhand rqt_bhand
```


---
###Recording Kinesthetic Demonstration as ROSBAGS:


--
###Converting ROSBAGS of Demonstrations to .mat files:
