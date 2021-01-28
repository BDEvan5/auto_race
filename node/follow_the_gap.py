#! /usr/bin/env python

from numpy.core.fromnumeric import cumprod
import rospy 
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

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
        self.aim_pub = rospy.Publisher('aim', Marker, queue_size=10)
        self.start_pub = rospy.Publisher('start', Marker, queue_size=10)
        self.end_pub = rospy.Publisher('end', Marker, queue_size=10)
        
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

        max_range = 3
        ranges = ranges[270:810] # only take the ones in front of vehicle

        ranges = self.preprocess_lidar(ranges, max_range)

        bubble_r = 0.1
        ranges = self.create_zero_bubble(ranges, bubble_r)
        
        start_i, end_i = self.find_max_gap(ranges)
        m = self.generate_marker(ranges, start_i, scan.angle_increment)
        m.color.r = 0
        m.color.g = 1
        m.scale.z = 0.6
        self.start_pub.publish(m)
        m = self.generate_marker(ranges, end_i, scan.angle_increment)
        m.color.r = 0
        m.color.b = 1
        m.scale.z = 0.6
        self.end_pub.publish(m)

        aim = self.find_best_point(start_i, end_i, ranges[start_i:end_i])

        m = self.generate_marker(ranges, aim, scan.angle_increment)
        self.aim_pub.publish(m)


        half_pt = len(ranges) /2
        steering_angle =  scan.angle_increment * (aim - half_pt)

        speed = self.max_speed * ranges[aim] / max_range * 0.7

        self.publish_drive_msg(speed, steering_angle)
        # self.publish_drive_msg(0, 0)

    def generate_marker(self, ranges, i, angle_increment):
        angle = -np.pi/2 + angle_increment * i
        range_val = ranges[i]
        m = Marker()
        m.header.frame_id = "laser"
        m.pose.position.x = range_val * np.cos(angle)
        m.pose.position.y = range_val * np.sin(angle)
        m.pose.position.z = 0
        m.pose.orientation.x = 0
        m.pose.orientation.y = 0
        m.pose.orientation.z = 0
        m.pose.orientation.w = 1
        m.type = m.ARROW
        m.action = m.ADD
        m.color.a = 1
        m.color.r = 1
        m.ns = "Goal Target"
        m.scale.x = 0.1
        m.scale.y = 0.1
        m.scale.z = 1

        return m




    def preprocess_lidar(self, ranges, max_range):
        ranges = [min(ran, max_range) for ran in ranges]
        
        # moving_avg
        # n = 3
        # cumsum = np.cumsum(np.insert(ranges, 0, 0))
        # proc_ranges = (cumsum[n:] - cumsum[:-n])/float(n)

        proc_ranges = ranges

        return proc_ranges

    def create_zero_bubble(self, input_vector, bubble_r):
        centre = np.argmin(input_vector)
        min_dist = input_vector[centre]
        input_vector[centre] = 0
        size = len(input_vector)

        current_idx = centre
        while(current_idx < size -1 and input_vector[current_idx] < (min_dist + bubble_r)):
            input_vector[current_idx] = 0
            current_idx += 1
        
        current_idx = centre
        while(current_idx > 0  and input_vector[current_idx] < (min_dist + bubble_r)):
            input_vector[current_idx] = 0
            current_idx -= 1

        return input_vector
        


    def find_max_gap(self, input_vector):
        max_start = 0
        max_size = 0

        current_idx = 0
        size = len(input_vector)

        while current_idx < size:
            current_start = current_idx
            current_size = 0
            while current_idx< size and input_vector[current_idx] > 1:
                current_size += 1
                current_idx += 1
            if current_size > max_size:
                max_start = current_start
                max_size = current_size
                current_size = 0
            current_idx += 1
        if current_size > max_size:
            max_start = current_start
            max_size = current_size

        return max_start, max_start + max_size - 1
        




    def find_best_point(self, start_i, end_i, ranges):
        # return best index to goto
        mid_i = (start_i + end_i) /2
        best_i = np.argmax(ranges)  
        best_i = (mid_i + (best_i + start_i)) /2

        return int(best_i)





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
