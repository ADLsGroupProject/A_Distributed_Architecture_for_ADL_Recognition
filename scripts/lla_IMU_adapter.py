#!/usr/bin/env python

## 
# @file lla_IMU_adapter.py 
# @author Davide Piccinini <piccio98dp\@gmail.com>
# @date 24 March 2021
# @brief This file contains the left lower arm IMU adapter.
# @note See @c adapter_template.py for more documentation.


import rospy
import csv
import os
from group_project.msg import ImuSample


samplePub = None

sample = ImuSample()


def llaAdapter():
    rate = rospy.Rate(33.33)

    while not rospy.is_shutdown():
        for row in imuData:
            sample.label = row[0]
            sample.timestamp = int(row[1])
            sample.quaternions = [float(row[2]), float(row[3]), float(row[4]), float(row[5])]
            sample.accelerations = [float(row[6]), float(row[7]), float(row[8])]
            sample.angular_velocities = [float(row[9]), float(row[10]), float(row[11])]

            samplePub.publish(sample)

            rate.sleep()

        break


if __name__ == "__main__":
    try:
        rospy.init_node('lla_IMU_adapter')

        script_path = os.path.abspath(__file__) 
        path_list = script_path.split(os.sep)
        script_directory = path_list[0:len(path_list)-2]
        csv_path = rospy.get_param("lla_csv_path")
        path = "/".join(script_directory) + "/" + csv_path
        csvFile = open(path, 'r')
        imuData = csv.reader(csvFile)

        samplePub = rospy.Publisher("lla_IMU_data", ImuSample, queue_size=1)

        llaAdapter()
        
    except rospy.ROSInterruptException:
        pass