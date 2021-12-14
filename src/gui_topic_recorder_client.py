#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import record_ros
from record_ros.srv import *
import shlex
from psutil import Popen
from tkinter import *
from tkinter import messagebox

def record_ros_client(command):
    rospy.wait_for_service('/record/cmd')
    try:
        record_ros = rospy.ServiceProxy('/record/cmd', String_cmd)
        resp1 = record_ros(command)
        return resp1.res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def record():
    command = 'record'
    record_ros_client(command)

def stop():
    command = 'stop'
    record_ros_client(command)

if __name__ == "__main__":
    
    root = Tk()
    root.title("RosBag Recorder")
    root.geometry("250x75+1300+0")

    B1 = Button(root, text = "Record", command = record)
    B1.place(x = 30,y = 20)

    B2 = Button(root, text = "Stop", command = stop)
    B2.place(x = 160,y = 20)

    root.mainloop()