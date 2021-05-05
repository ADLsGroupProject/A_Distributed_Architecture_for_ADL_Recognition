#!/usr/bin/env python

## 
# @file rla_IMU_adapter.py 
# @author Davide Piccinini <piccio98dp\@gmail.com>
# @date 24 March 2021
# @brief This file contains the right lower arm IMU adapter.
# @note See @c adapter_template.py for more documentation.


import rospy
import csv
import os
from group_project.msg import ImuSample


samplePub = None

sample = ImuSample()


def rlaAdapter():
    rate = rospy.Rate(33.33)

    print("Right lower arm IMU adapter: Running...")

    for row in imuData:
        if rospy.is_shutdown():
            break

        sample.label = row[0]
        sample.timestamp = int(row[1])
        sample.quaternions = [float(row[2]), float(row[3]), float(row[4]), float(row[5])]
        sample.accelerations = [float(row[6]), float(row[7]), float(row[8])]
        sample.angular_velocities = [float(row[9]), float(row[10]), float(row[11])]

        samplePub.publish(sample)

        rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node('rla_IMU_adapter')

        script_path = os.path.abspath(__file__) 
        path_list = script_path.split(os.sep)
        script_directory = path_list[0:len(path_list)-2]
        csv_path = rospy.get_param("/rla_csv_path")
        path = "/".join(script_directory) + "/" + csv_path
        csvFile = open(path, 'r')
        imuData = csv.reader(csvFile)

        samplePub = rospy.Publisher("rla_IMU_data", ImuSample, queue_size=1)

        rlaAdapter()
        
    except rospy.ROSInterruptException:
        pass