#! /usr/bin/env python

from numpy.core.fromnumeric import cumprod
import rospy 
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan

import numpy as np


class FollowTheGap:
    def __init__(self):
        self.map = None
        self.cur_scan = None
        self.cur_odom = None
        
        drive_topic = rospy.get_param("f1tenth_simulator/drive_topic")

        odom_topic = rospy.get_param("f1tenth_simulator/odom_topic")
        map_topic = rospy.get_param("f1tenth_simulator/map_topic")
        scan_topic = rospy.get_param("f1tenth_simulator/scan_topic")
        # imu_topic = rospy.get_param("f1tenth_simulator/imu_topic")

        self.max_speed = rospy.get_param("f1tenth_simulator/max_speed")
        self.max_steer = rospy.get_param("f1tenth_simulator/max_steering_angle")

        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        
        # self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        # self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        self.scan_sub = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback, queue_size=1)

        rospy.spin()

    def odom_callback(self, odom):
        # self.cur_odom = odom
        pass
        # speed = self.max_speed/2
        # steer = 0

        # self.publish_drive_msg(speed, steer)

    def map_callback(self, map):
        # self.map = map
        pass


    def scan_callback(self, scan):
        ranges = scan.ranges
        # possibly do something with ranges outside of the possible driving area?
        proc_ranges = self.preprocess_lidar(ranges)

        # possibly have multiple bubbles? for ranges below a certain threshold
        i_min = np.argmin(proc_ranges)
        bubble = 5
        #todo: add in min/max to stop search outside of range
        for i in range(i_min-bubble, i_min+bubble):
            proc_ranges[i] = 0

        start_i, end_i = self.find_max_gap(proc_ranges)

        aim = self.find_best_point(start_i, end_i, proc_ranges[start_i:end_i])
        
        half_pt = len(scan.ranges) /2
        steering_angle =  scan.angle_increment * (aim - half_pt)


        speed = self.max_speed / 4

        self.publish_drive_msg(speed, steering_angle)


    def preprocess_lidar(self, ranges):
        max_range = 5
        ranges = [min(ran, 5) for ran in ranges]
        
        # moving_avg
        n = 3
        cumsum = np.cumsum(np.insert(ranges, 0, 0))
        proc_ranges = (cumsum[n:] - cumsum[:-n])/float(n)

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        # return start and end index of max gap
        gap_arr = [0]
        starts = [0]
        ends = []
        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] > 0:
                # not in gap - counting gap size
                gap_arr[-1] += 1
            elif gap_arr[-1] != 0:
                # first gap bit
                gap_arr.append(0)
                ends.append(i)
                starts.append(i)
            else:
                # in the gap
                starts[-1] = i
        ends.append(len(free_space_ranges))
        
        i_gap = np.argmax(gap_arr)

        i_start = starts[i_gap]
        i_end = ends[i_gap]

        return i_start, i_end


    def find_best_point(self, start_i, end_i, ranges):
        # return best index to goto
        best_i = (start_i + end_i) /2

        return best_i 





    def publish_drive_msg(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.drive = AckermannDrive()

        msg.drive.speed = speed
        msg.drive.steering_angle = steer

        self.drive_pub.publish(msg)




if __name__ =="__main__":
    try:
        rospy.init_node('follow_the_gap', anonymous=True)

        rospy.loginfo("Node follow_the_gap has been created")

        ftg = FollowTheGap()


    except rospy.ROSInterruptException:
        pass
