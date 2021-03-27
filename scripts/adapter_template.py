#!/usr/bin/env python

## 
# @file adapter_template.py 
# @author Davide Piccinini <piccio98dp\@gmail.com>
# @date 23 March 2021
# @brief This file contains the template used to simulate an IMU 
# sending data on a topic.


import rospy
import csv
import os
from group_project.msg import ImuSample


## ROS publisher
samplePub = None

## IMU sample message definition
sample = ImuSample()


##
# @brief Publish a sample on the topic at a fixed frequency.
# 
# The sensors publish data with a 30 ms sampling time, so the frequency
# will be 33.33 Hz. This node sends the contents of a csv file sample
# by sample using the @c ImuSample.msg custom message format.
#
# @note Once the node has published all the file's contents, it will
#       shutdown.
def adapter():
    # Publish a sample every 30 ms
    rate = rospy.Rate(33.33)

    for row in imuData:
        # If ROS is shut down, stop sending data
        if rospy.is_shutdown():
            break

        # Convert the data to their correct type
        sample.label = row[0]
        sample.timestamp = int(row[1])
        sample.quaternions = [float(row[2]), float(row[3]), float(row[4]), float(row[5])]
        sample.accelerations = [float(row[6]), float(row[7]), float(row[8])]
        sample.angular_velocities = [float(row[9]), float(row[10]), float(row[11])]

        # Publish the sample on the topic
        samplePub.publish(sample)

        rate.sleep()


if __name__ == "__main__":
    try:
        # Initialize the node
        rospy.init_node('general_adapter')

        # Open and read the csv file
        script_path = os.path.abspath(__file__) 
        path_list = script_path.split(os.sep)
        script_directory = path_list[0:len(path_list)-2]
        csv_path = "/data/volunteer_01/IMUs/back.csv"
        path = "/".join(script_directory) + "/" + csv_path
        csvFile = open(path, 'r')
        imuData = csv.reader(csvFile)

        # Initialize the data publisher
        samplePub = rospy.Publisher("general_IMU_data", ImuSample, queue_size=1)

        # Start sending data
        adapter()
        
    except rospy.ROSInterruptException:
        pass