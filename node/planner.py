#! /usr/bin/env python

import rospy 
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry

import numpy as np


class Planner:
    def __init__(self):
        
        pd_t = rospy.get_param("planner/planner_drive_topic")
        odom_t = rospy.get_param("planner/odom_topic")

        self.max_speed = rospy.get_param("planner/max_speed")
        self.max_steer = rospy.get_param("planner/max_steering_angle")

        self.drive_pub = rospy.Publisher(pd_t, AckermannDriveStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber(odom_t, Odometry, self.odom_callback, queue_size=1)

        rospy.spin()

        rospy.loginfo("Node Planner has been initialised")

    def odom_callback(self, odom):
        rospy.loginfo("Odom callback engaged")
        speed = self.max_speed/2
        steer = (np.random.random() - 0.5) * self.max_steer

        rospy.loginfo("Node called - speed: " + str(speed) + ", steer: " + str(steer))

        self.publish_drive_msg(speed, steer)

    def publish_drive_msg(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.drive = AckermannDrive()

        msg.drive.speed = speed
        msg.drive.steering_angle = steer

        self.drive_pub.publish(msg)




if __name__ =="__main__":
    try:
        rospy.init_node('planner', anonymous=True)

        rospy.loginfo("Node Planner has been created")

        p = Planner()


    except rospy.ROSInterruptException:
        pass



