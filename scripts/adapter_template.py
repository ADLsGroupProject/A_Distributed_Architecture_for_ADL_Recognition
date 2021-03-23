#!/usr/bin/env python

## @package 
# 

import rospy
import csv
import os
from group_project.msg import ImuSample


## Publisher
samplePub = None

## IMU sample
sample = ImuSample()


##
# 
def adapter():
    # Publish a sample every 30 ms
    rate = rospy.Rate(33.33)

    # Keep sending imu samples on the topic until the file is finished
    while not rospy.is_shutdown():
        for row in imuData:
            sample.label = row[0]
            sample.timestamp = int(row[1])
            sample.quaternions = [float(row[2]), float(row[3]), float(row[4]), float(row[5])]
            sample.accelerations = [float(row[6]), float(row[7]), float(row[8])]
            sample.angular_velocities = [float(row[9]), float(row[10]), float(row[11])]

            # Publish the sample on the topic
            samplePub.publish(sample)

            rate.sleep()

        break


if __name__ == "__main__":
    try:
        # Initialize the node
        rospy.init_node('adapter')

        # Open the csv file
        script_path = os.path.abspath(__file__) 
        path_list = script_path.split(os.sep)
        script_directory = path_list[0:len(path_list)-2]
        csv_path = "/data/volunteer_01/IMUs/back.csv"
        path = "/".join(script_directory) + "/" + csv_path
        csvFile = open(path, 'r')
        imuData = csv.reader(csvFile)

        # Initialize the data publisher
        samplePub = rospy.Publisher("backIMU_data", ImuSample, queue_size=1)

        adapter()
        
    except rospy.ROSInterruptException:
        pass