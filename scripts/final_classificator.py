#!/usr/bin/env python

## 
# @file final_classificator.py 
# @author Davide Piccinini <piccio98dp\@gmail.com>
# @date 24 March 2021
# @brief This file represents the final component of the architecture,
# which publishes the system's classification of a sliding window.


import rospy
import os
import threading
from datetime import datetime
from statistics import mode
from std_msgs.msg import String
from group_project.msg import ImuSample, SystemClassification


## ROS publisher
classificationPub = None

## Back sensor module ROS subscriber
backSub = None
## Left lower arm sensor module ROS subscriber
llaSub = None
## Left upper arm sensor module ROS subscriber
luaSub = None
## Right lower arm sensor module ROS subscriber
rlaSub = None
## Right thigh sensor module ROS subscriber
rtSub = None
## Right upper arm sensor module ROS subscriber
ruaSub = None

## Threading event
backReceived = threading.Event()
## Threading event
llaReceived = threading.Event()
## Threading event
luaReceived = threading.Event()
## Threading event
rlaReceived = threading.Event()
## Threading event
rtReceived = threading.Event()
## Threading event
ruaReceived = threading.Event()

## Labels list
labels = []

## Sliding window overlap parameter
overlap = 0

## System classification message definition
sysClass = SystemClassification()


##
# @brief 
def backCallback(label):
    global labels
    global backReceived

    if not backReceived.is_set():
        # Save the label
        labels.append(label.data)

        # Set the flag to true
        backReceived.set()


##
# @brief 
def llaCallback(label):
    global labels
    global llaReceived

    if not llaReceived.is_set():
        # Save the label
        labels.append(label.data)
        
        # Set the flag to true
        llaReceived.set()


##
# @brief 
def luaCallback(label):
    global labels
    global luaReceived

    if not luaReceived.is_set():
        # Save the label
        labels.append(label.data)
        
        # Set the flag to true
        luaReceived.set()


##
# @brief 
def rlaCallback(label):
    global labels
    global rlaReceived

    if not rlaReceived.is_set():
        # Save the label
        labels.append(label.data)
        
        # Set the flag to true
        rlaReceived.set()


##
# @brief 
def rtCallback(label):
    global labels
    global rtReceived

    if not rtReceived.is_set():
        # Save the label
        labels.append(label.data)
        
        # Set the flag to true
        rtReceived.set()


##
# @brief 
def ruaCallback(label):
    global labels
    global ruaReceived

    if not ruaReceived.is_set():
        # Save the label
        labels.append(label.data)
        
        # Set the flag to true
        ruaReceived.set()


##
# @brief 
def classify():
    # global labels
    # global sysClass
    # global classificationPub
    # global backReceived, llaReceived, luaReceived, rlaReceived, rtReceived, ruaReceived

    # Get the starting timestamp of the first sliding window
    now = datetime.now()
    midnight = now.replace(hour=0, minute=0, second=0, microsecond=0)
    starting_timestamp = round((now.timestamp() - midnight.timestamp()) * 1000)

    while not rospy.is_shutdown():
        # Wait until all the 6 labels have been received
        backReceived.wait()
        llaReceived.wait()
        luaReceived.wait()
        rlaReceived.wait()
        rtReceived.wait()
        ruaReceived.wait()

        # Get the ending timestamp of the sliding window
        ending_timestamp = round((datetime.now().timestamp() - midnight.timestamp()) * 1000)

        # Create the message
        sysClass.starting_timestamp = starting_timestamp
        sysClass.ending_timestamp = ending_timestamp
        sysClass.classification = mode(labels)

        # Publish the message
        classificationPub.publish(sysClass)

        # Compute the next starting_timestamp (timedelta should be more or less constant)
        timedelta = ending_timestamp - starting_timestamp
        starting_timestamp = ending_timestamp - round(timedelta * overlap)

        # Clear the list
        labels.clear()

        # Set all the flags back to false
        backReceived.clear()
        llaReceived.clear()
        luaReceived.clear()
        rlaReceived.clear()
        rtReceived.clear()
        ruaReceived.clear()

        rospy.spin()


##
# @brief Program initialization.
if __name__ == "__main__":
    try:
        # Initialize the node
        rospy.init_node('final_classificator')

        # Initialize the classification publisher
        classificationPub = rospy.Publisher("system_classification", SystemClassification, queue_size=1)

        # Initialize the subscribers
        backSub = rospy.Subscriber("sensor/back_IMU_label", String, backCallback)
        llaSub = rospy.Subscriber("sensor/lla_IMU_label", String, llaCallback)
        luaSub = rospy.Subscriber("sensor/lua_IMU_label", String, luaCallback)
        rlaSub = rospy.Subscriber("sensor/rla_IMU_label", String, rlaCallback)
        rtSub = rospy.Subscriber("sensor/rt_IMU_label", String, rtCallback)
        ruaSub = rospy.Subscriber("sensor/rua_IMU_label", String, ruaCallback)

        # Get the sliding window overlap parameter
        overlap = float(rospy.get_param("/sliding_window_overlap"))

        # Start classifying the sliding windows
        classify()
        
    except rospy.ROSInterruptException:
        pass