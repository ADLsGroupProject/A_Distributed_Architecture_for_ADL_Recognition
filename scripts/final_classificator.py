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
import csv
from datetime import datetime
from statistics import mode, mean
from std_msgs.msg import String
from group_project.msg import ImuSample, Classification


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

## Starting timestamps list
startingTimestamps = []

## Ending timestamps list
endingTimestamps = []

## Sensor modules' confidence coefficients list
confCoeffs = []

## System classification message definition
sysClass = Classification()


##
# @brief Callback function for the @c sensor/back_IMU_label topic.
# @param message The message sent by the back sensor module. 
def backCallback(message):
    if not backReceived.is_set():
        # Save the starting timestamp
        startingTimestamps.append(message.starting_timestamp)

        # Save the ending timestamp
        endingTimestamps.append(message.ending_timestamp)

        # Save the label
        labels.append(message.label)

        # Save the confidence coefficient
        confCoeffs.append(message.confidence_coefficient)

        # Set the flag to true
        backReceived.set()


##
# @brief Callback function for the @c sensor/lla_IMU_label topic.
# @param message The message sent by the left lower arm sensor module. 
def llaCallback(message):
    if not llaReceived.is_set():
        # Save the starting timestamp
        startingTimestamps.append(message.starting_timestamp)

        # Save the ending timestamp
        endingTimestamps.append(message.ending_timestamp)

        # Save the label
        labels.append(message.label)

        # Save the confidence coefficient
        confCoeffs.append(message.confidence_coefficient)
        
        # Set the flag to true
        llaReceived.set()


##
# @brief Callback function for the @c sensor/lua_IMU_label topic.
# @param message The message sent by the left upper arm sensor module. 
def luaCallback(message):
    if not luaReceived.is_set():
        # Save the starting timestamp
        startingTimestamps.append(message.starting_timestamp)

        # Save the ending timestamp
        endingTimestamps.append(message.ending_timestamp)

        # Save the label
        labels.append(message.label)

        # Save the confidence coefficient
        confCoeffs.append(message.confidence_coefficient)
        
        # Set the flag to true
        luaReceived.set()


##
# @brief Callback function for the @c sensor/rla_IMU_label topic.
# @param message The message sent by the right lower arm sensor module. 
def rlaCallback(message):
    if not rlaReceived.is_set():
        # Save the starting timestamp
        startingTimestamps.append(message.starting_timestamp)

        # Save the ending timestamp
        endingTimestamps.append(message.ending_timestamp)

        # Save the label
        labels.append(message.label)

        # Save the confidence coefficient
        confCoeffs.append(message.confidence_coefficient)
        
        # Set the flag to true
        rlaReceived.set()


##
# @brief Callback function for the @c sensor/rt_IMU_label topic.
# @param message The message sent by the right thigh sensor module. 
def rtCallback(message):
    if not rtReceived.is_set():
        # Save the starting timestamp
        startingTimestamps.append(message.starting_timestamp)

        # Save the ending timestamp
        endingTimestamps.append(message.ending_timestamp)

        # Save the label
        labels.append(message.label)

        # Save the confidence coefficient
        confCoeffs.append(message.confidence_coefficient)
        
        # Set the flag to true
        rtReceived.set()


##
# @brief Callback function for the @c sensor/rua_IMU_label topic.
# @param message The message sent by the right upper arm sensor module. 
def ruaCallback(message):
    if not ruaReceived.is_set():
        # Save the starting timestamp
        startingTimestamps.append(message.starting_timestamp)

        # Save the ending timestamp
        endingTimestamps.append(message.ending_timestamp)

        # Save the label
        labels.append(message.label)

        # Save the confidence coefficient
        confCoeffs.append(message.confidence_coefficient)
        
        # Set the flag to true
        ruaReceived.set()


##
# @brief Write the output on the csv file if the user specified it in 
# the launch file.
# @param starting_timestamp The starting timestamp (ms since midnight) 
# of the sliding window.
# @param ending_timestamp The ending timestamp (ms since midnight) 
# of the sliding window. 
# @param label The label representing the final output of the system.
# @param confidence_coefficient The number in the range [0, 1] 
# representing the confidence on the output label: high value means 
# high confidence.
def writeOnCsv(startingTimestamp, endingTimestamp, label, confidenceCoefficient):
    if writeOutput:
        outputCsv.writerow([str(startingTimestamp), str(endingTimestamp), label, str(confidenceCoefficient)])
        csvFile.flush()


##
# @brief Output the system's classification on the @c system_classification
# topic and eventually on a user-defined csv file.
#
# Wait for all the sensor modules to publish their message then verify 
# that both the starting and ending timestamps are coherent: if so, get
# the starting and ending timestamps of the overall sliding window, 
# compute the average confidence coefficient, create the message, send
# it on the topic and start the thread to write it on the csv file;
# otherwise, simply notify that an invalid classification has been 
# detected.
def classify():
    print("Final classificator: Running...")

    while not rospy.is_shutdown():
        # Set the two coherence bool variables to false
        startCoherence = False
        endCoherence = False

        # Wait until all the 6 messages have been received
        backReceived.wait()
        llaReceived.wait()
        luaReceived.wait()
        rlaReceived.wait()
        rtReceived.wait()
        ruaReceived.wait()

        # Check the coherence of the timestamps
        if (max(startingTimestamps) - min(startingTimestamps)) <= tolerance:
            startCoherence = True
        if (max(endingTimestamps) - min(endingTimestamps)) <= tolerance:
            endCoherence = True

        # If both lists of timestamps are coherent, compute and send 
        # the output message
        if startCoherence and endCoherence:
            # Find the most occurring label
            finalLabel = mode(labels)

            # Compute the confidence coefficient of the system's classification
            # Get the indexes corresponding to the most occurring label
            indexes = [i for i,x in enumerate(labels) if x == finalLabel]
            # Make a list containing only the relevant sensor modules' coefficients
            finalCoeffs = []
            for v in indexes:
                finalCoeffs.append(confCoeffs[v])
            # Finally append the final classificator's confidence coefficient 
            finalCoeffs.append(len(indexes) / 6)

            # Create the message
            sysClass.starting_timestamp = min(startingTimestamps)
            sysClass.ending_timestamp = max(endingTimestamps)
            sysClass.label = finalLabel
            sysClass.confidence_coefficient = round(mean(finalCoeffs), 3)

            # Publish the message
            classificationPub.publish(sysClass)

            # Write the message on the csv file
            thread = threading.Thread(target=writeOnCsv, args=(sysClass.starting_timestamp, sysClass.ending_timestamp, sysClass.label, sysClass.confidence_coefficient, ))
            thread.start()
        else:
            print("Final classificator: The timestamps of the received data aren't coherent, no classification produced.\n")

        # Clear the lists
        startingTimestamps.clear()
        endingTimestamps.clear()
        labels.clear()
        confCoeffs.clear()

        # Set all the flags back to false
        backReceived.clear()
        llaReceived.clear()
        luaReceived.clear()
        rlaReceived.clear()
        rtReceived.clear()
        ruaReceived.clear()


if __name__ == "__main__":
    try:
        # Initialize the node
        rospy.init_node('final_classificator')

        # Initialize the classification publisher
        classificationPub = rospy.Publisher("system_classification", Classification, queue_size=1)

        # Initialize the subscribers
        backSub = rospy.Subscriber("classificator/back_IMU_classification", Classification, backCallback)
        llaSub = rospy.Subscriber("classificator/lla_IMU_classification", Classification, llaCallback)
        luaSub = rospy.Subscriber("classificator/lua_IMU_classification", Classification, luaCallback)
        rlaSub = rospy.Subscriber("classificator/rla_IMU_classification", Classification, rlaCallback)
        rtSub = rospy.Subscriber("classificator/rt_IMU_classification", Classification, rtCallback)
        ruaSub = rospy.Subscriber("classificator/rua_IMU_classification", Classification, ruaCallback)

        # Open the csv file for writing if the user specified so
        writeOutput = rospy.get_param("write_on_csv")
        if writeOutput:
            script_path = os.path.abspath(__file__) 
            path_list = script_path.split(os.sep)
            script_directory = path_list[0:len(path_list)-2]
            csv_path = rospy.get_param("output_csv_path")
            path = "/".join(script_directory) + "/" + csv_path
            csvFile = open(path, 'w', newline='')
            outputCsv = csv.writer(csvFile, delimiter=',', quotechar='"')

        # Retrieve the tolerance parameter
        tolerance = rospy.get_param("tolerance_in_ms")

        # Start classifying the sliding windows
        classify()
        
    except rospy.ROSInterruptException:
        pass